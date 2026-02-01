//
// Abstract IMU backend interface
//

#pragma once

#include "imu_types.h"
#include "esp_err.h"

namespace imu
{

class IMU;

class Backend
{
public:
    virtual ~Backend() = default;

    virtual esp_err_t start(const SpiConfig& spi,
                           uint32_t gyro_interval_us,
                           uint32_t accel_interval_us,
                           uint32_t quat_interval_us) = 0;
    virtual esp_err_t stop() = 0;

    virtual bool has_fusion() const = 0;
    virtual uint32_t max_gyro_rate_hz() const = 0;
    virtual uint32_t max_accel_rate_hz() const = 0;
    virtual uint32_t max_quat_rate_hz() const = 0;

    virtual const char* name() const = 0;

protected:
    void push_accel(const AccelData& data) const;
    void push_gyro(const GyroData& data) const;
    void push_quat(const QuatData& data) const;
    void push_euler(const EulerData& data) const;
};

} // namespace imu
