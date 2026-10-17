#include "imu.h"

namespace imu {

IMU& IMU::instance() {
    static IMU instance;
    return instance;
}

esp_err_t IMU::register_backend(Backend* backend) {
    if (!backend) return ESP_ERR_INVALID_ARG;
    if (backend_) return ESP_ERR_INVALID_STATE;
    backend_ = backend;
    return ESP_OK;
}

Samples IMU::samples() const {
    portENTER_CRITICAL(&samples_lock_);
    const auto result = samples_;
    portEXIT_CRITICAL(&samples_lock_);
    return result;
}

esp_err_t IMU::set_observer(const SampleObserver& observer) {
    if (backend_) return ESP_ERR_INVALID_STATE;
    observer_ = observer;
    return ESP_OK;
}

void IMU::push_accel_from_backend(const AccelData& data) {
    portENTER_CRITICAL(&samples_lock_);
    samples_.accel = data;
    portEXIT_CRITICAL(&samples_lock_);
    if (observer_.accel) observer_.accel(observer_.context, data);
}

void IMU::push_gyro_from_backend(const GyroData& data) {
    portENTER_CRITICAL(&samples_lock_);
    samples_.gyro = data;
    portEXIT_CRITICAL(&samples_lock_);
    if (observer_.gyro) observer_.gyro(observer_.context, data);
}

void IMU::push_quat_from_backend(const QuatData& data) {
    portENTER_CRITICAL(&samples_lock_);
    samples_.quat = data;
    portEXIT_CRITICAL(&samples_lock_);
    if (observer_.quat) observer_.quat(observer_.context, data);
}

void IMU::push_euler_from_backend(const EulerData& data) {
    portENTER_CRITICAL(&samples_lock_);
    samples_.euler = data;
    portEXIT_CRITICAL(&samples_lock_);
    if (observer_.euler) observer_.euler(observer_.context, data);
}

void Backend::push_accel(const AccelData& data) const { IMU::instance().push_accel_from_backend(data); }
void Backend::push_gyro(const GyroData& data) const { IMU::instance().push_gyro_from_backend(data); }
void Backend::push_quat(const QuatData& data) const { IMU::instance().push_quat_from_backend(data); }
void Backend::push_euler(const EulerData& data) const { IMU::instance().push_euler_from_backend(data); }

} // namespace imu
