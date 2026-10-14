//
// IMU facade singleton implementation
//

#include "imu.h"
#include "blackboard/blackboard.h"
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

void IMU::push_accel_from_backend(const AccelData& data)
{
    blackboard::publish(data);
}

void IMU::push_gyro_from_backend(const GyroData& data)
{
    blackboard::publish(data);
}

void IMU::push_quat_from_backend(const QuatData& data)
{
    blackboard::publish(data);
}

void IMU::push_euler_from_backend(const EulerData& data)
{
    blackboard::publish(data);
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
