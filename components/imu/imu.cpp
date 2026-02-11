//
// IMU facade singleton implementation
//

#include "imu.h"
#include "telemetry.h"
#include "esp_log.h"

namespace imu
{

static const char* TAG = "imu";

IMU& IMU::instance()
{
    static IMU inst;
    return inst;
}

esp_err_t IMU::register_backend(Backend* b)
{
    if (!b) {
        ESP_LOGE(TAG, "register_backend called with nullptr");
        return ESP_ERR_INVALID_ARG;
    }
    if (backend_) {
        ESP_LOGE(TAG, "cannot register backend, one is already set");
        return ESP_ERR_INVALID_STATE;
    }
    backend_ = b;
    ESP_LOGI(TAG, "registered backend: %s", b->name());
    return ESP_OK;
}

AccelData IMU::get_accel() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return accel_data_;
}

GyroData IMU::get_gyro() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return gyro_data_;
}

QuatData IMU::get_quat() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return quat_data_;
}

EulerData IMU::get_euler() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return euler_data_;
}

void IMU::push_accel_from_backend(const AccelData& data)
{
    GyroData gyro_snap{};
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        accel_data_ = data;
        gyro_snap = gyro_data_;
    }
    accel_valid_.store(data.valid, std::memory_order_relaxed);

    // if (data.valid && gyro_snap.valid) {
    //     telemetry::AccelGyroTelemetry ag = {
    //         .sample_time_us = static_cast<uint32_t>(data.timestamp_us),
    //         .gyro_x_rads = gyro_snap.x_rads,
    //         .gyro_y_rads = gyro_snap.y_rads,
    //         .gyro_z_rads = gyro_snap.z_rads,
    //         .accel_x_ms2 = data.x_ms2,
    //         .accel_y_ms2 = data.y_ms2,
    //         .accel_z_ms2 = data.z_ms2,
    //     };
    //     telemetry::publish(ag);
    // }
}

void IMU::push_gyro_from_backend(const GyroData& data)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        gyro_data_ = data;
    }
    gyro_valid_.store(data.valid, std::memory_order_relaxed);
}

void IMU::push_quat_from_backend(const QuatData& data)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        quat_data_ = data;
    }
    quat_valid_.store(data.valid, std::memory_order_relaxed);
}

void IMU::push_euler_from_backend(const EulerData& data)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        euler_data_ = data;
    }
    euler_valid_.store(data.valid, std::memory_order_relaxed);

    if (data.valid) {
        telemetry::AttitudeTelemetry att = {
            .roll_deg = data.roll_deg,
            .pitch_deg = data.pitch_deg,
            .yaw_deg = data.yaw_deg
        };
        telemetry::publish(att);
    }
}

// Backend protected method implementations
void Backend::push_accel(const AccelData& data) const
{
    IMU::instance().push_accel_from_backend(data);
}

void Backend::push_gyro(const GyroData& data) const
{
    IMU::instance().push_gyro_from_backend(data);
}

void Backend::push_quat(const QuatData& data) const
{
    IMU::instance().push_quat_from_backend(data);
}

void Backend::push_euler(const EulerData& data) const
{
    IMU::instance().push_euler_from_backend(data);
}

} // namespace imu
