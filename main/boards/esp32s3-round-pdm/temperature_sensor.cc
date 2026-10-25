#include "temperature_sensor.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#define TAG "TemperatureSensor"

#define AHT20_ADDR   0x38
#define BMP280_ADDR      0x76
#define BMP280_ADDR_ALT  0x77

// AHT20 命令
#define AHT20_CMD_INIT        0xBE
#define AHT20_CMD_TRIGGER     0xAC
#define AHT20_CMD_SOFT_RESET  0xBA

// BMP280 寄存器
#define BMP280_REG_ID         0xD0
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_STATUS     0xF3
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_PRESS_MSB  0xF7
#define BMP280_REG_CALIB      0x88

// ---------------------------------------------------------------------------
// AHT20
// ---------------------------------------------------------------------------
class TemperatureSensor::Aht20 : public I2cDevice {
public:
    explicit Aht20(i2c_master_bus_handle_t bus) : I2cDevice(bus, AHT20_ADDR) {
        // 软复位后等 20ms 让芯片稳定
        uint8_t reset_cmd = AHT20_CMD_SOFT_RESET;
        i2c_master_transmit(i2c_device_, &reset_cmd, 1, 100);
        vTaskDelay(pdMS_TO_TICKS(30));

        // 0xBE 0x08 0x00：初始化/校准
        uint8_t init_cmd[3] = {AHT20_CMD_INIT, 0x08, 0x00};
        esp_err_t err = i2c_master_transmit(i2c_device_, init_cmd, 3, 100);
        ok_ = (err == ESP_OK);
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "AHT20 %s", ok_ ? "就绪" : "初始化失败");
    }

    bool Ok() const { return ok_; }

    /** 返回是否读到有效数据；温度写 t，湿度写 h */
    bool Read(float& t, float& h) {
        if (!ok_) {
            return false;
        }

        // 触发测量：0xAC 0x33 0x00
        uint8_t trigger[3] = {AHT20_CMD_TRIGGER, 0x33, 0x00};
        if (i2c_master_transmit(i2c_device_, trigger, 3, 100) != ESP_OK) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(80));   // AHT20 测量约需 75ms

        // 读 6 字节原始数据
        uint8_t data[6] = {0};
        if (i2c_master_receive(i2c_device_, data, 6, 100) != ESP_OK) {
            return false;
        }

        // 状态字节的 bit7 是忙标志，置位说明还没测完
        if (data[0] & 0x80) {
            ESP_LOGW(TAG, "AHT20 仍在忙");
            return false;
        }

        // 20 位湿度 + 20 位温度
        uint32_t raw_humidity = ((uint32_t)data[1] << 12) |
                                ((uint32_t)data[2] << 4) |
                                ((uint32_t)data[3] >> 4);
        uint32_t raw_temperature = (((uint32_t)data[3] & 0x0F) << 16) |
                                   ((uint32_t)data[4] << 8) |
                                   (uint32_t)data[5];

        h = (float)raw_humidity * 100.0f / 1048576.0f;
        t = (float)raw_temperature * 200.0f / 1048576.0f - 50.0f;
        return true;
    }

private:
    bool ok_ = false;
};

// ---------------------------------------------------------------------------
// BMP280
// ---------------------------------------------------------------------------
class TemperatureSensor::Bmp280 : public I2cDevice {
public:
    Bmp280(i2c_master_bus_handle_t bus, uint8_t addr) : I2cDevice(bus, addr) {
        uint8_t id = 0;
        if (i2c_master_transmit_receive(i2c_device_, (uint8_t[]){BMP280_REG_ID}, 1, &id, 1, 100) != ESP_OK) {
            ok_ = false;
            ESP_LOGW(TAG, "BMP280@0x%02X 读 ID 失败", addr);
            return;
        }
        // BMP280 的 ID 是 0x58，BME280 是 0x60（兼容）；BMP180 是 0x55（寄存器完全不同，不支持）
        if (id != 0x58 && id != 0x60) {
            ESP_LOGW(TAG, "0x%02X 上的芯片 ID=0x%02X，不是 BMP280/BME280%s", addr, id,
                     id == 0x55 ? "（这是 BMP180，寄存器不同，需要另写驱动）" : "");
            ok_ = false;
            return;
        }

        // 读校准系数（24 字节）
        uint8_t calib[24] = {0};
        if (i2c_master_transmit_receive(i2c_device_, (uint8_t[]){BMP280_REG_CALIB}, 1, calib, 24, 100) != ESP_OK) {
            ok_ = false;
            return;
        }
        dig_T1_ = (uint16_t)(calib[0] | (calib[1] << 8));
        dig_T2_ = (int16_t)(calib[2] | (calib[3] << 8));
        dig_T3_ = (int16_t)(calib[4] | (calib[5] << 8));
        dig_P1_ = (uint16_t)(calib[6] | (calib[7] << 8));
        dig_P2_ = (int16_t)(calib[8] | (calib[9] << 8));
        dig_P3_ = (int16_t)(calib[10] | (calib[11] << 8));
        dig_P4_ = (int16_t)(calib[12] | (calib[13] << 8));
        dig_P5_ = (int16_t)(calib[14] | (calib[15] << 8));
        dig_P6_ = (int16_t)(calib[16] | (calib[17] << 8));
        dig_P7_ = (int16_t)(calib[18] | (calib[19] << 8));
        dig_P8_ = (int16_t)(calib[20] | (calib[21] << 8));
        dig_P9_ = (int16_t)(calib[22] | (calib[23] << 8));

        // 配置：正常模式，温度过采样 x2（0x40），压力过采样 x16（0x05），不睡眠
        uint8_t cfg[2] = {BMP280_REG_CTRL_MEAS, 0x57};
        i2c_master_transmit(i2c_device_, cfg, 2, 100);

        ok_ = true;
        ESP_LOGI(TAG, "BMP280 就绪 (地址=0x%02X, ID=0x%02X)", addr, id);
    }

    bool Ok() const { return ok_; }

    bool Read(float& pressure_hpa, float& temperature_c) {
        if (!ok_) {
            return false;
        }

        uint8_t data[6] = {0};
        if (i2c_master_transmit_receive(i2c_device_, (uint8_t[]){BMP280_REG_PRESS_MSB}, 1, data, 6, 100) != ESP_OK) {
            return false;
        }

        int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
        int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);

        // 温度补偿（Bosch 官方算法）
        int32_t var1_t = ((((adc_T >> 3) - ((int32_t)dig_T1_ << 1))) * (int32_t)dig_T2_) >> 11;
        int32_t var2_t = (((((adc_T >> 4) - (int32_t)dig_T1_) *
                            ((adc_T >> 4) - (int32_t)dig_T1_)) >> 12) * (int32_t)dig_T3_) >> 14;
        t_fine_ = var1_t + var2_t;
        temperature_c = (float)((t_fine_ * 5 + 128) >> 8) / 100.0f;

        // 压力补偿
        int64_t var1 = (int64_t)t_fine_ - 128000;
        int64_t var2 = var1 * var1 * (int64_t)dig_P6_;
        var2 = var2 + ((var1 * (int64_t)dig_P5_) << 17);
        var2 = var2 + (((int64_t)dig_P4_) << 35);
        var1 = ((var1 * var1 * (int64_t)dig_P3_) >> 8) + ((var1 * (int64_t)dig_P2_) << 12);
        var1 = ((((int64_t)1) << 47) + var1) * (int64_t)dig_P1_ >> 33;

        if (var1 == 0) {
            return false;   // 校准系数异常
        }

        int64_t p = 1048576 - adc_P;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = ((int64_t)dig_P9_ * (p >> 13) * (p >> 13)) >> 25;
        var2 = ((int64_t)dig_P8_ * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7_) << 4);

        pressure_hpa = (float)p / 256.0f / 100.0f;
        return true;
    }

private:
    bool ok_ = false;
    int32_t t_fine_ = 0;

    uint16_t dig_T1_ = 0;
    int16_t dig_T2_ = 0, dig_T3_ = 0;
    uint16_t dig_P1_ = 0;
    int16_t dig_P2_ = 0, dig_P3_ = 0, dig_P4_ = 0, dig_P5_ = 0;
    int16_t dig_P6_ = 0, dig_P7_ = 0, dig_P8_ = 0, dig_P9_ = 0;
};

// ---------------------------------------------------------------------------
// TemperatureSensor
// ---------------------------------------------------------------------------
TemperatureSensor::TemperatureSensor(i2c_master_bus_handle_t bus) {
    // 先扫一遍总线，把所有应答的地址打出来——比猜地址靠谱
    std::string found;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(bus, addr, 20) == ESP_OK) {
            char buf[8];
            snprintf(buf, sizeof(buf), "0x%02X ", addr);
            found += buf;
        }
    }
    ESP_LOGI(TAG, "I2C 总线扫描，有应答的地址: %s", found.empty() ? "（无）" : found.c_str());

    aht20_ = std::make_unique<Aht20>(bus);

    // BMP280 的地址由 SDO 脚决定：接 GND = 0x76，接 VCC = 0x77。两个都试。
    uint8_t bmp_addr = 0;
    if (i2c_master_probe(bus, BMP280_ADDR, 20) == ESP_OK) {
        bmp_addr = BMP280_ADDR;
    } else if (i2c_master_probe(bus, BMP280_ADDR_ALT, 20) == ESP_OK) {
        bmp_addr = BMP280_ADDR_ALT;
    }
    if (bmp_addr != 0) {
        bmp280_ = std::make_unique<Bmp280>(bus, bmp_addr);
    } else {
        ESP_LOGW(TAG, "0x76 / 0x77 都没有应答——BMP280 没接上，或模块上的气压芯片是别的型号");
    }
}

TemperatureSensor::~TemperatureSensor() = default;

bool TemperatureSensor::Update() {
    bool any_ok = false;

    float t = 0.0f, h = 0.0f;
    if (aht20_ && aht20_->Ok() && aht20_->Read(t, h)) {
        temperature_ = t;
        humidity_ = h;
        has_temperature_ = true;
        has_humidity_ = true;
        any_ok = true;
    }

    float p = 0.0f, bt = 0.0f;
    if (bmp280_ && bmp280_->Ok() && bmp280_->Read(p, bt)) {
        pressure_ = p;
        if (!has_temperature_) {
            // AHT20 挂了就用 BMP280 的温度顶上
            temperature_ = bt;
            has_temperature_ = true;
        }
        has_pressure_ = true;
        any_ok = true;
    }

    return any_ok;
}

std::string TemperatureSensor::GetJson() const {
    std::string json = "{";
    bool first = true;

    auto add = [&json, &first](const char* key, float value, int decimals) {
        if (!first) {
            json += ",";
        }
        first = false;
        char buf[32];
        snprintf(buf, sizeof(buf), "%.*f", decimals, value);
        json += "\"";
        json += key;
        json += "\":";
        json += buf;
    };

    if (has_temperature_) {
        add("temperature_c", temperature_, 1);
    }
    if (has_humidity_) {
        add("humidity_pct", humidity_, 1);
    }
    if (has_pressure_) {
        add("pressure_hpa", pressure_, 1);
    }
    json += "}";

    if (first) {
        return "{\"error\":\"no sensor available\"}";
    }
    return json;
}
