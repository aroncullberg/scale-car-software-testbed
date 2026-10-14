//
// IMU data types - application-friendly, backend-agnostic
//

#pragma once

#include <cstdint>
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "blackboard/blackboard.h"

namespace imu
{

enum class Accuracy : uint8_t {
    UNRELIABLE = 0,
    LOW = 1,
    MEDIUM = 2,
    HIGH = 3
};

struct AccelData {
    float x_ms2;
    float y_ms2;
    float z_ms2;
    Accuracy accuracy;
    uint64_t timestamp_us;
    bool valid;
};

struct GyroData {
    float x_rads;
    float y_rads;
    float z_rads;
    Accuracy accuracy;
    uint64_t timestamp_us;
    bool valid;
};

struct QuatData {
    float w;
    float x;
    float y;
    float z;
    float accuracy_rad;
    Accuracy accuracy;
    uint64_t timestamp_us;
    bool valid;
};

struct EulerData {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float accuracy_rad;
    Accuracy accuracy;
    uint64_t timestamp_us;
    bool valid;
};

struct SpiConfig {
    gpio_num_t mosi_pin;
    gpio_num_t miso_pin;
    gpio_num_t sclk_pin;
    gpio_num_t cs_pin;
    gpio_num_t int_pin;
    gpio_num_t rst_pin;
    spi_host_device_t host;
    uint32_t clock_speed_hz;
};

} // namespace imu

namespace blackboard
{
template <> struct Topic<imu::AccelData> { static constexpr TopicId id = TopicId::accel; };
template <> struct Topic<imu::GyroData>  { static constexpr TopicId id = TopicId::gyro;  };
template <> struct Topic<imu::QuatData>  { static constexpr TopicId id = TopicId::quat;  };
template <> struct Topic<imu::EulerData> { static constexpr TopicId id = TopicId::euler; };
} // namespace blackboard
