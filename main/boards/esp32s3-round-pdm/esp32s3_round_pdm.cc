#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "led/single_led.h"
#include "temperature_sensor.h"
#include "mcp_server.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_st7789.h>
#include <driver/spi_common.h>

#define TAG "Esp32S3RoundPdmBoard"

// ESP32-S3-N16R8 + ST7735S 1.8" 128x160 + NS4168 功放 / LMD2718 PDM 麦克风
class Esp32S3RoundPdmBoard : public WifiBoard {
private:
    Button boot_button_;
    Button k1_button_;
    Button k2_button_;
    Button k3_button_;
    Button k4_button_;
    LcdDisplay* display_;
    i2c_master_bus_handle_t sensor_i2c_bus_ = nullptr;
    TemperatureSensor* sensor_ = nullptr;

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
        // ST7735S 用 IDF 内置的 ST7789 驱动点亮：两者命令集兼容，
        // 工程里其他的 ST7735 板（bread-compact-esp32-lcd 的 128x160 档）也是这么点的。
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new SpiLcdDisplay(panel_io, panel,
                                     DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                     DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        ESP_LOGI(TAG, "LCD 就绪 %dx%d", DISPLAY_WIDTH, DISPLAY_HEIGHT);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        // K1：摆摊叫卖（立即叫卖一句）
        k1_button_.OnClick([this]() {
            Application::GetInstance().TriggerStallCall();
        });

        // K2：音量 +
        k2_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            int volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });
        k2_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        // K3：音量 -
        k3_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            int volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });
        k3_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });

        // K4：开始/停止说话；长按重新配网（换 WiFi 时不用再拆壳按 BOOT）
        k4_button_.OnClick([this]() {
            Application::GetInstance().ToggleChatState();
        });
        k4_button_.OnLongPress([this]() {
            EnterWifiConfigMode();
        });
    }

    // 环境传感器：AHT20（温湿度）+ BMP280（气压），共用一条 I2C 总线
    void InitializeSensor() {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = I2C_NUM_0;
        bus_cfg.sda_io_num = SENSOR_I2C_SDA_GPIO;
        bus_cfg.scl_io_num = SENSOR_I2C_SCL_GPIO;
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = true;

        esp_err_t err = i2c_new_master_bus(&bus_cfg, &sensor_i2c_bus_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2C 总线初始化失败: %s，传感器不可用", esp_err_to_name(err));
            sensor_i2c_bus_ = nullptr;
            return;
        }

        sensor_ = new TemperatureSensor(sensor_i2c_bus_);
        ESP_LOGI(TAG, "环境传感器已初始化（SDA=%d, SCL=%d）",
                 (int)SENSOR_I2C_SDA_GPIO, (int)SENSOR_I2C_SCL_GPIO);
    }

    // 注册 MCP 工具，让语音能问到室温、湿度、气压
    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();

        if (sensor_ != nullptr) {
            mcp.AddTool(
                "self.sensor.read_environment",
                "读取桌面环境的温度和湿度、气压。当用户问「现在室温多少」「房间潮不潮」"
                "「今天气压怎么样」这类问题时调用。返回 JSON，包含 temperature_c（摄氏度）、"
                "humidity_pct（相对湿度百分比）、pressure_hpa（气压百帕）；"
                "某个传感器没接上时对应字段缺失。",
                PropertyList(),
                [this](const PropertyList& properties) -> ReturnValue {
                    if (!sensor_->Update()) {
                        return std::string("{\"error\":\"传感器读取失败，检查接线\"}");
                    }
                    return sensor_->GetJson();
                });
        }
    }

public:
    Esp32S3RoundPdmBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        k1_button_(K1_BUTTON_GPIO),
        k2_button_(K2_BUTTON_GPIO),
        k3_button_(K3_BUTTON_GPIO),
        k4_button_(K4_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeSensor();
        InitializeTools();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplexPdm audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_DIN);
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
};

DECLARE_BOARD(Esp32S3RoundPdmBoard);
