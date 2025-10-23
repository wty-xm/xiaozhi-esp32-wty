#ifndef MAIN_PROTOCOLS_YP0X_UART_H_
#define MAIN_PROTOCOLS_YP0X_UART_H_

#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>
#include <functional>

struct Yp0xResult {
    uint8_t info = 0;
    uint16_t systolic = 0;
    uint8_t diastolic = 0;
    uint8_t mean = 0;
    uint16_t pulse = 0;
    uint8_t reserved = 0;
};

class Yp0xHost {
public:
    using VersionCallback = std::function<void(uint8_t major, uint8_t minor, uint8_t patch, uint8_t model)>;
    using ResultCallback = std::function<void(const Yp0xResult&)>;

    Yp0xHost() = default;
    ~Yp0xHost();

    esp_err_t Init(uart_port_t port,
                   gpio_num_t tx,
                   gpio_num_t rx,
                   int baud = 115200,
                   int rx_buffer_size = 256);

    void Start();
    void Stop();

    esp_err_t StartMeasure();
    esp_err_t StopMeasure();
    esp_err_t QueryVersion();
    esp_err_t QueryLastMeasurement();

    void OnVersion(VersionCallback cb) { on_version_ = std::move(cb); }
    void OnResult(ResultCallback cb) { on_result_ = std::move(cb); }

private:
    static constexpr uint8_t kHeader = 0xFF;
    static constexpr size_t kFrameBufferSize = 64;

    enum class RxState : uint8_t {
        kWaitHeader,
        kWaitLength,
        kReadBody,
    };

    esp_err_t SendCommand(uint8_t command_id,
                          const uint8_t* data,
                          size_t data_len,
                          bool need_ack = false);

    static void RxTask(void* arg);
    void RxLoop();

    void ResetParser();
    void FeedByte(uint8_t byte);
    uint8_t ComputeChecksum(const uint8_t* frame) const;

    uart_port_t port_ = UART_NUM_MAX;
    gpio_num_t tx_pin_ = GPIO_NUM_NC;
    gpio_num_t rx_pin_ = GPIO_NUM_NC;
    int baudrate_ = 115200;

    TaskHandle_t rx_task_ = nullptr;

    VersionCallback on_version_;
    ResultCallback on_result_;

    RxState rx_state_ = RxState::kWaitHeader;
    uint8_t expected_length_ = 0;
    uint8_t buffer_[kFrameBufferSize] = {0};
    size_t buffer_pos_ = 0;
};

#endif  // MAIN_PROTOCOLS_YP0X_UART_H_
