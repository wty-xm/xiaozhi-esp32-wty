#include "protocols/yp0x_uart.h"

#include <esp_check.h>
#include <esp_log.h>

#include <algorithm>
#include <cstring>

namespace {
constexpr const char* kTag = "Yp0xHost";
constexpr TickType_t kReadTimeoutTicks = pdMS_TO_TICKS(20);
constexpr uint8_t kAckBitMask = 0x40;
constexpr uint8_t kCommandIdMask = 0x3F;
}  // namespace

Yp0xHost::~Yp0xHost() {
    Stop();
    if (port_ != UART_NUM_MAX && uart_is_driver_installed(port_)) {
        uart_driver_delete(port_);
    }
}

esp_err_t Yp0xHost::Init(uart_port_t port,
                         gpio_num_t tx,
                         gpio_num_t rx,
                         int baud,
                         int rx_buffer_size) {
    if (port < UART_NUM_0 || port >= UART_NUM_MAX) {
        ESP_LOGE(kTag, "Invalid UART port: %d", static_cast<int>(port));
        return ESP_ERR_INVALID_ARG;
    }
    port_ = port;
    tx_pin_ = tx;
    rx_pin_ = rx;
    baudrate_ = baud;

    uart_config_t config = {};
    config.baud_rate = baud;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;

    if (!uart_is_driver_installed(port_)) {
        ESP_RETURN_ON_ERROR(
            uart_driver_install(port_, rx_buffer_size, 0, 0, nullptr, 0), kTag,
            "Failed to install UART driver");
    }

    ESP_RETURN_ON_ERROR(uart_param_config(port_, &config), kTag,
                        "Failed to set UART param");
    ESP_RETURN_ON_ERROR(uart_set_pin(port_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE),
                        kTag, "Failed to set UART pins");

    ResetParser();
    return ESP_OK;
}

void Yp0xHost::Start() {
    if (rx_task_ || port_ == UART_NUM_MAX) {
        return;
    }
    BaseType_t rc = xTaskCreatePinnedToCore(
        &Yp0xHost::RxTask, "yp0x_uart_rx", 4096, this, 5, &rx_task_,
        tskNO_AFFINITY);
    if (rc != pdPASS) {
        ESP_LOGE(kTag, "Failed to create RX task");
        rx_task_ = nullptr;
    }
}

void Yp0xHost::Stop() {
    if (rx_task_) {
        TaskHandle_t task = rx_task_;
        rx_task_ = nullptr;
        vTaskDelete(task);
    }
}

esp_err_t Yp0xHost::StartMeasure() {
    uint8_t data = 0x01;
    return SendCommand(0x10, &data, 1);
}

esp_err_t Yp0xHost::StopMeasure() {
    uint8_t data = 0x00;
    return SendCommand(0x10, &data, 1);
}

esp_err_t Yp0xHost::QueryVersion() {
    uint8_t data = 0x00;
    return SendCommand(0x13, &data, 1);
}

esp_err_t Yp0xHost::QueryLastMeasurement() {
    uint8_t data = 0x01;
    return SendCommand(0x13, &data, 1);
}

esp_err_t Yp0xHost::SendCommand(uint8_t command_id,
                                const uint8_t* data,
                                size_t data_len,
                                bool need_ack) {
    if (port_ == UART_NUM_MAX) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data_len > (kFrameBufferSize - 5)) {
        ESP_LOGE(kTag, "Command data too large: %u",
                 static_cast<unsigned>(data_len));
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t length = static_cast<uint8_t>(1 + data_len + 1);
    uint8_t frame[kFrameBufferSize] = {0};
    size_t frame_len = 0;

    frame[frame_len++] = kHeader;
    frame[frame_len++] = length;

    uint8_t command = command_id & kCommandIdMask;
    if (need_ack) {
        command |= kAckBitMask;
    }

    frame[frame_len++] = command;
    if (data_len > 0 && data) {
        std::memcpy(&frame[frame_len], data, data_len);
        frame_len += data_len;
    }

    const uint8_t checksum = ComputeChecksum(frame);
    frame[frame_len++] = checksum;

    const int written =
        uart_write_bytes(port_, reinterpret_cast<const char*>(frame), frame_len);
    if (written != static_cast<int>(frame_len)) {
        ESP_LOGW(kTag, "UART write truncated (%d/%u)", written,
                 static_cast<unsigned>(frame_len));
        return ESP_FAIL;
    }
    return ESP_OK;
}

void Yp0xHost::RxTask(void* arg) {
    auto* self = static_cast<Yp0xHost*>(arg);
    if (self) {
        self->RxLoop();
    }
    vTaskDelete(nullptr);
}

void Yp0xHost::RxLoop() {
    uint8_t rx_buf[64];
    while (true) {
        const int len = uart_read_bytes(port_, rx_buf, sizeof(rx_buf),
                                        kReadTimeoutTicks);
        if (len <= 0) {
            if (rx_task_ == nullptr) {
                break;
            }
            continue;
        }
        for (int i = 0; i < len; ++i) {
            FeedByte(rx_buf[i]);
        }
        if (rx_task_ == nullptr) {
            break;
        }
    }
}

void Yp0xHost::ResetParser() {
    rx_state_ = RxState::kWaitHeader;
    expected_length_ = 0;
    buffer_pos_ = 0;
    std::fill(std::begin(buffer_), std::end(buffer_), 0);
}

void Yp0xHost::FeedByte(uint8_t byte) {
    switch (rx_state_) {
        case RxState::kWaitHeader:
            if (byte == kHeader) {
                buffer_[0] = byte;
                buffer_pos_ = 1;
                rx_state_ = RxState::kWaitLength;
            }
            break;
        case RxState::kWaitLength:
            if (byte >= 2 && byte <= 0x20) {
                buffer_[buffer_pos_++] = byte;
                expected_length_ = byte;
                rx_state_ = RxState::kReadBody;
            } else {
                ESP_LOGW(kTag, "Invalid frame length: 0x%02X", byte);
                ResetParser();
            }
            break;
        case RxState::kReadBody:
            if (buffer_pos_ >= kFrameBufferSize) {
                ESP_LOGW(kTag, "Frame too large, dropping");
                ResetParser();
                break;
            }
            buffer_[buffer_pos_++] = byte;
            if (buffer_pos_ == static_cast<size_t>(2 + expected_length_)) {
                const uint8_t checksum = ComputeChecksum(buffer_);
                const uint8_t received_checksum = buffer_[buffer_pos_ - 1];
                if (checksum == received_checksum) {
                    const uint8_t command_full = buffer_[2];
                    const uint8_t command_id = command_full & kCommandIdMask;
                    const uint8_t* data = &buffer_[3];
                    const size_t data_len = expected_length_ >= 2
                                                ? static_cast<size_t>(expected_length_ - 2)
                                                : 0;

                    if (command_id == 0x31 && data_len >= 8 && on_result_) {
                        Yp0xResult result;
                        result.info = data[0];
                        result.systolic =
                            static_cast<uint16_t>(data[1]) |
                            (static_cast<uint16_t>(data[2]) << 8);
                        result.diastolic = data[3];
                        result.mean = data[4];
                        result.pulse =
                            static_cast<uint16_t>(data[5]) |
                            (static_cast<uint16_t>(data[6]) << 8);
                        result.reserved = data[7];
                        on_result_(result);
                    } else if (command_id == 0x39 && data_len >= 4 &&
                               on_version_) {
                        const uint8_t major = data[0] & 0x7F;
                        const uint8_t minor = data[1] & 0x7F;
                        const uint8_t patch = data[2] & 0x7F;
                        const uint8_t model = data[3];
                        on_version_(major, minor, patch, model);
                    } else {
                        ESP_LOGD(kTag,
                                 "Unhandled frame, cmd=0x%02X len=%u checksum ok",
                                 command_id, static_cast<unsigned>(data_len));
                    }
                } else {
                    ESP_LOGW(kTag,
                             "Checksum mismatch (computed=0x%02X recv=0x%02X)",
                             checksum, received_checksum);
                }
                ResetParser();
            }
            break;
    }
}

uint8_t Yp0xHost::ComputeChecksum(const uint8_t* frame) const {
    const uint8_t length = frame[1];
    uint8_t checksum = 0;
    for (uint8_t i = 1; i < static_cast<uint8_t>(length + 1); ++i) {
        checksum = static_cast<uint8_t>(checksum + frame[i]);
    }
    if (checksum == kHeader) {
        checksum = static_cast<uint8_t>(checksum - 1);
    }
    return checksum;
}
