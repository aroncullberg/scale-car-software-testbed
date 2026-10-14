//
// IMU facade singleton - unified interface for IMU data access
//

#pragma once

#include "esp_err.h"
#include "imu_types.h"
#include "imu_backend.h"

namespace imu
{

class Backend;

// Producer-side facade: the backend registers here and feeds samples in, which
// are published to the blackboard. Consumers read IMU data from the blackboard
// (blackboard::latest<GyroData>() etc.), not from this class.
class IMU
{
public:
    static IMU& instance();

    esp_err_t register_backend(Backend* b);
    const Backend* backend() const { return backend_; }

    bool has_fusion() const { return backend_ && backend_->has_fusion(); }

    IMU(const IMU&) = delete;
    IMU& operator=(const IMU&) = delete;

private:
    friend class Backend;
    IMU() = default;

    void push_accel_from_backend(const AccelData& data);
    void push_gyro_from_backend(const GyroData& data);
    void push_quat_from_backend(const QuatData& data);
    void push_euler_from_backend(const EulerData& data);

    Backend* backend_{nullptr};
};

} // namespace imu
