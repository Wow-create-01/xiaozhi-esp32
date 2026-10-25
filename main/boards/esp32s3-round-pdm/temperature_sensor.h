#ifndef _TEMPERATURE_SENSOR_H_
#define _TEMPERATURE_SENSOR_H_

#include <memory>
#include <string>

#include <driver/i2c_master.h>
#include "i2c_device.h"

/**
 * AHT20 + BMP280 组合传感器。
 *
 * AHT20（地址 0x38）：温湿度，用标准的两字节读命令，不需要初始化寄存器。
 * BMP280（地址 0x76 或 0x77）：气压 + 温度，需要校准系数补偿。
 *
 * 两个芯片共用一条 I2C 总线（SDA=GPIO21, SCL=GPIO2），地址不冲突。
 */
class TemperatureSensor {
public:
    TemperatureSensor(i2c_master_bus_handle_t bus);
    ~TemperatureSensor();

    /** 读一次并缓存。返回是否至少有一个传感器成功。 */
    bool Update();

    /** 上报给 MCP 工具的 JSON 字符串 */
    std::string GetJson() const;

private:
    class Aht20;
    class Bmp280;

    std::unique_ptr<Aht20> aht20_;
    std::unique_ptr<Bmp280> bmp280_;

    // 缓存最近一次读数
    bool has_temperature_ = false;
    bool has_humidity_ = false;
    bool has_pressure_ = false;
    float temperature_ = 0.0f;   // 摄氏度
    float humidity_ = 0.0f;      // 百分比
    float pressure_ = 0.0f;      // hPa
};

#endif  // _TEMPERATURE_SENSOR_H_
