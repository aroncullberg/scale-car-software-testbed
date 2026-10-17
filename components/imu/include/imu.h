#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "imu_types.h"
#include "imu_backend.h"

namespace imu {

struct Samples {
    AccelData accel{};
    GyroData gyro{};
    QuatData quat{};
    EulerData euler{};
};

struct SampleObserver {
    void* context = nullptr;
    void (*accel)(void*, const AccelData&) = nullptr;
    void (*gyro)(void*, const GyroData&) = nullptr;
    void (*quat)(void*, const QuatData&) = nullptr;
    void (*euler)(void*, const EulerData&) = nullptr;
};

// The backend updates these samples; consumers read a consistent copy.
class IMU {
public:
    static IMU& instance();
    esp_err_t register_backend(Backend* backend);
    esp_err_t set_observer(const SampleObserver& observer);
    Samples samples() const;
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
    mutable portMUX_TYPE samples_lock_ = portMUX_INITIALIZER_UNLOCKED;
    Samples samples_{};
    SampleObserver observer_{};
};

} // namespace imu
