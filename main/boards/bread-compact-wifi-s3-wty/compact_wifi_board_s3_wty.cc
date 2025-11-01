#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "protocols/yp0x_uart.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#include <optional>
#include <string>
#include <cstdio>
#include <stdexcept>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <cJSON.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
     (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                 0x04, 0x12, 0x14, 0x1f},
     14, 0},
    {0xf1,
     (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                 0x0C, 0x1A, 0x14, 0x1E},
     14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif

#define TAG "CompactWifiBoardS3Wty"

class CompactWifiBoardS3Wty : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay* display_ = nullptr;
    Yp0xHost blood_pressure_host_;
    bool blood_pressure_initialized_ = false;
    std::optional<Yp0xResult> last_bp_result_;
    std::optional<Yp0xResult> last_announced_bp_result_;
    bool bp_auto_report_pending_ = false;
    SemaphoreHandle_t bp_result_sem_ = nullptr;
    struct {
        bool valid = false;
        uint8_t major = 0;
        uint8_t minor = 0;
        uint8_t patch = 0;
        uint8_t model = 0;
    } bp_version_;

    std::string FormatBpResult(const Yp0xResult& result) const {
        char buffer[96];
        snprintf(buffer, sizeof(buffer),
                 "info=0x%02X systolic=%u diastolic=%u mean=%u pulse=%u extra=0x%02X",
                 result.info, result.systolic, result.diastolic, result.mean,
                 result.pulse, result.reserved);
        return std::string(buffer);
    }

    void AnnounceBloodPressure(const Yp0xResult& result) {
        char text[160];
        snprintf(text, sizeof(text),
                 "Latest measurement: systolic %u mmHg, diastolic %u mmHg, heart rate %u bpm.",
                 result.systolic, result.diastolic, result.pulse);

        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "jsonrpc", "2.0");
        cJSON_AddStringToObject(root, "method", "client.callbacks.on_message");

        cJSON* params = cJSON_CreateObject();
        cJSON* messages = cJSON_CreateArray();

        cJSON* message = cJSON_CreateObject();
        cJSON_AddStringToObject(message, "role", "assistant");
        cJSON* content = cJSON_CreateArray();
        cJSON* text_obj = cJSON_CreateObject();
        cJSON_AddStringToObject(text_obj, "type", "text");
        cJSON_AddStringToObject(text_obj, "text", text);
        cJSON_AddItemToArray(content, text_obj);
        cJSON_AddItemToObject(message, "content", content);
        cJSON_AddItemToArray(messages, message);

        cJSON_AddItemToObject(params, "messages", messages);
        cJSON_AddItemToObject(root, "params", params);

        char* payload = cJSON_PrintUnformatted(root);
        Application::GetInstance().SendMcpMessage(payload);
        cJSON_free(payload);
        cJSON_Delete(root);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;

#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };
        panel_config.vendor_config = &gc9107_vendor_config;
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new SpiLcdDisplay(panel_io, panel,
                                     DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                     DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                     DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    void InitializeBloodPressureInterface() {
        if (blood_pressure_initialized_) {
            return;
        }
        if (bp_result_sem_ == nullptr) {
            bp_result_sem_ = xSemaphoreCreateBinary();
            if (bp_result_sem_ == nullptr) {
                ESP_LOGE(TAG, "Failed to create BP result semaphore");
            }
        } else {
            while (xSemaphoreTake(bp_result_sem_, 0) == pdTRUE) {
            }
        }

        esp_err_t err = blood_pressure_host_.Init(BP_UART_PORT, BP_UART_TX_PIN, BP_UART_RX_PIN, BP_UART_BAUDRATE);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to init blood pressure UART: %s", esp_err_to_name(err));
            return;
        }

        blood_pressure_host_.OnVersion([this](uint8_t major, uint8_t minor, uint8_t patch, uint8_t model) {
            bp_version_.valid = true;
            bp_version_.major = major;
            bp_version_.minor = minor;
            bp_version_.patch = patch;
            bp_version_.model = model;
            ESP_LOGI(TAG, "BP version: %u.%u.%u (model 0x%02X)", major, minor, patch, model);
        });

        blood_pressure_host_.OnResult([this](const Yp0xResult& result) {
            last_bp_result_ = result;
            ESP_LOGI(TAG,
                     "BP measurement info=0x%02X SYS=%u DIA=%u MAP=%u PULSE=%u EXT=0x%02X",
                     result.info, result.systolic, result.diastolic, result.mean,
                     result.pulse, result.reserved);

            const bool should_announce =
                bp_auto_report_pending_ ||
                !last_announced_bp_result_.has_value() ||
                result.systolic != last_announced_bp_result_->systolic ||
                result.diastolic != last_announced_bp_result_->diastolic ||
                result.pulse != last_announced_bp_result_->pulse;

            if (should_announce) {
                bp_auto_report_pending_ = false;
                AnnounceBloodPressure(result);
                last_announced_bp_result_ = result;
            } else {
                last_announced_bp_result_ = result;
            }

            if (bp_result_sem_) {
                xSemaphoreGive(bp_result_sem_);
            }
        });

        blood_pressure_host_.Start();
        blood_pressure_initialized_ = true;

        auto& mcp_server = McpServer::GetInstance();

        mcp_server.AddTool(
            "self.bp.start_measure", "Start blood pressure measurement", PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (!blood_pressure_initialized_) {
                    throw std::runtime_error("Blood pressure UART not initialized");
                }
                const esp_err_t start_err = blood_pressure_host_.StartMeasure();
                if (start_err != ESP_OK) {
                    throw std::runtime_error(std::string("Failed to start measurement: ") + esp_err_to_name(start_err));
                }
                bp_auto_report_pending_ = true;
                return true;
            });

        mcp_server.AddTool(
            "self.bp.stop_measure", "Stop measurement / put meter to standby", PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (!blood_pressure_initialized_) {
                    throw std::runtime_error("Blood pressure UART not initialized");
                }
                const esp_err_t stop_err = blood_pressure_host_.StopMeasure();
                if (stop_err != ESP_OK) {
                    throw std::runtime_error(std::string("Failed to stop measurement: ") + esp_err_to_name(stop_err));
                }
                return true;
            });

        mcp_server.AddTool(
            "self.bp.query_version", "Query meter firmware version", PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (!blood_pressure_initialized_) {
                    throw std::runtime_error("Blood pressure UART not initialized");
                }
                const esp_err_t query_err = blood_pressure_host_.QueryVersion();
                if (query_err != ESP_OK) {
                    throw std::runtime_error(std::string("Failed to query version: ") + esp_err_to_name(query_err));
                }
                return true;
            });

        mcp_server.AddTool(
            "self.bp.query_last", "Query last measurement result", PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (!blood_pressure_initialized_) {
                    throw std::runtime_error("Blood pressure UART not initialized");
                }

                bp_auto_report_pending_ = true;
                if (bp_result_sem_ != nullptr) {
                    while (xSemaphoreTake(bp_result_sem_, 0) == pdTRUE) {
                    }
                }

                const esp_err_t query_err = blood_pressure_host_.QueryLastMeasurement();
                if (query_err != ESP_OK) {
                    throw std::runtime_error(std::string("Failed to query measurement: ") + esp_err_to_name(query_err));
                }

                bool received = false;
                if (bp_result_sem_ != nullptr) {
                    received = xSemaphoreTake(bp_result_sem_, pdMS_TO_TICKS(1500)) == pdTRUE;
                    if (!received) {
                        ESP_LOGW(TAG, "Timed out waiting for BP query result");
                    }
                } else {
                    vTaskDelay(pdMS_TO_TICKS(200));
                }

                if (!last_bp_result_.has_value()) {
                    throw std::runtime_error("No measurement available");
                }

                if (received) {
                    bp_auto_report_pending_ = false;
                }

                return FormatBpResult(last_bp_result_.value());
            });

        mcp_server.AddTool(
            "self.bp.get_cached_version", "Get cached version info (if any)", PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (!blood_pressure_initialized_) {
                    throw std::runtime_error("Blood pressure UART not initialized");
                }
                if (!bp_version_.valid) {
                    return std::string("No version info");
                }
                char buffer[64];
                snprintf(buffer, sizeof(buffer), "%u.%u.%u (model 0x%02X)",
                         bp_version_.major, bp_version_.minor, bp_version_.patch, bp_version_.model);
                return std::string(buffer);
            });

        mcp_server.AddTool(
            "self.bp.get_cached_result", "Get cached measurement (if any)", PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (!blood_pressure_initialized_) {
                    throw std::runtime_error("Blood pressure UART not initialized");
                }
                if (!last_bp_result_.has_value()) {
                    return std::string("No measurement available");
                }
                return FormatBpResult(last_bp_result_.value());
            });
    }

public:
    CompactWifiBoardS3Wty() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            if (auto backlight = GetBacklight()) {
                backlight->RestoreBrightness();
            }
        }
        InitializeBloodPressureInterface();
    }

    virtual Led* GetLed() override {
        if (BUILTIN_LED_GPIO == GPIO_NUM_NC) {
            return nullptr;
        }
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual Camera* GetCamera() override {
        return nullptr;
    }
};

DECLARE_BOARD(CompactWifiBoardS3Wty);
