//
// IMU facade singleton - unified interface for IMU data access
//

#pragma once

#include <atomic>
#include <mutex>

#include "esp_err.h"
#include "imu_types.h"
#include "imu_backend.h"

namespace imu
{

class Backend;

class IMU
{
public:
    static IMU& instance();

    esp_err_t register_backend(Backend* b);
    const Backend* backend() const { return backend_; }

    // Thread-safe data access
    AccelData get_accel() const;
    GyroData get_gyro() const;
    QuatData get_quat() const;
    EulerData get_euler() const;

    // Fast validity checks (atomic, no mutex)
    bool has_valid_accel() const { return accel_valid_.load(std::memory_order_relaxed); }
    bool has_valid_gyro() const { return gyro_valid_.load(std::memory_order_relaxed); }
    bool has_valid_quat() const { return quat_valid_.load(std::memory_order_relaxed); }
    bool has_valid_euler() const { return euler_valid_.load(std::memory_order_relaxed); }

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

    mutable std::mutex data_mutex_;
    AccelData accel_data_{};
    GyroData gyro_data_{};
    QuatData quat_data_{};
    EulerData euler_data_{};

    std::atomic<bool> accel_valid_{false};
    std::atomic<bool> gyro_valid_{false};
    std::atomic<bool> quat_valid_{false};
    std::atomic<bool> euler_valid_{false};
};

} // namespace imu
