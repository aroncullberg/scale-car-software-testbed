//
// BNO08x backend implementation
//

#include "bno08x.h"
#include "imu.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace imu
{

static const char* TAG = "imu-bno08x";

BNO08xBackend::BNO08xBackend(const Config& cfg)
    : cfg_(cfg)
{
}

BNO08xBackend::~BNO08xBackend()
{
    stop();
    delete sensor_;
    sensor_ = nullptr;
}

esp_err_t BNO08xBackend::start(const SpiConfig& spi,
                               uint32_t gyro_interval_us,
                               uint32_t accel_interval_us,
                               uint32_t quat_interval_us)
{
    if (running_) {
        ESP_LOGW(TAG, "already running");
        return ESP_OK;
    }

    // Create BNO08x config from our SpiConfig
    bno08x_config_t bno_cfg(
        spi.host,
        spi.mosi_pin,
        spi.miso_pin,
        spi.sclk_pin,
        spi.cs_pin,
        spi.int_pin,
        spi.rst_pin,
        spi.clock_speed_hz,
        cfg_.install_isr_service
    );

    ESP_LOGI(TAG, "attempting ot create senosr instantce");

    // Create sensor instance
    sensor_ = new BNO08x(bno_cfg);

    ESP_LOGI(TAG, "sensor initialized");

    if (!sensor_->initialize()) {
        ESP_LOGE(TAG, "failed to initialize BNO08x sensor");
        delete sensor_;
        sensor_ = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BNO08x initialized successfully");



    // Register with facade
    esp_err_t err = IMU::instance().register_backend(this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to register backend: %s", esp_err_to_name(err));
        delete sensor_;
        sensor_ = nullptr;
        return err;
    }

    // Enable accelerometer report
    if (!sensor_->rpt.accelerometer.enable(accel_interval_us)) {
        ESP_LOGE(TAG, "failed to enable accelerometer report");
        return ESP_FAIL;
    }
    sensor_->rpt.accelerometer.register_cb([this]() { handle_accel(); });
    ESP_LOGI(TAG, "accelerometer enabled at %lu us interval", accel_interval_us);

    // Enable calibrated gyro report
    if (!sensor_->rpt.cal_gyro.enable(gyro_interval_us)) {
        ESP_LOGE(TAG, "failed to enable gyro report");
        return ESP_FAIL;
    }
    sensor_->rpt.cal_gyro.register_cb([this]() { handle_gyro(); });
    ESP_LOGI(TAG, "gyro enabled at %lu us interval", gyro_interval_us);

    // Enable rotation vector report (mag-referenced for absolute heading)
    if (quat_interval_us > 0) {
        if (!sensor_->rpt.rv.enable(quat_interval_us)) {
            ESP_LOGE(TAG, "failed to enable rotation vector report");
            return ESP_FAIL;
        }
        sensor_->rpt.rv.register_cb([this]() { handle_quat(); });
        ESP_LOGI(TAG, "rotation vector enabled at %lu us interval", quat_interval_us);
    }

    running_ = true;
    ESP_LOGI(TAG, "BNO08x backend started");
    return ESP_OK;
}

esp_err_t BNO08xBackend::stop()
{
    if (!running_ || !sensor_) {
        return ESP_OK;
    }

    sensor_->disable_all_reports();
    running_ = false;

    ESP_LOGI(TAG, "BNO08x backend stopped");
    return ESP_OK;
}

void BNO08xBackend::handle_accel()
{
    if (!sensor_->rpt.accelerometer.has_new_data()) {
        return;
    }

    bno08x_accel_t data = sensor_->rpt.accelerometer.get();

    AccelData accel{};
    accel.x_ms2 = data.x;
    accel.y_ms2 = data.y;
    accel.z_ms2 = data.z;
    accel.accuracy = convert_accuracy(data.accuracy);
    accel.timestamp_us = esp_timer_get_time();
    accel.valid = true;

    push_accel(accel);
}

void BNO08xBackend::handle_gyro()
{
    if (!sensor_->rpt.cal_gyro.has_new_data()) {
        return;
    }

    bno08x_gyro_t data = sensor_->rpt.cal_gyro.get();

    GyroData gyro{};
    gyro.x_rads = data.x;
    gyro.y_rads = data.y;
    gyro.z_rads = data.z;
    gyro.accuracy = convert_accuracy(data.accuracy);
    gyro.timestamp_us = esp_timer_get_time();
    gyro.valid = true;

    push_gyro(gyro);
}

void BNO08xBackend::handle_quat()
{
    if (!sensor_->rpt.rv.has_new_data()) {
        return;
    }

    bno08x_quat_t quat_data = sensor_->rpt.rv.get_quat();
    bno08x_euler_angle_t euler_data = sensor_->rpt.rv.get_euler();

    uint64_t now = esp_timer_get_time();

    // Push quaternion
    QuatData quat{};
    quat.w = quat_data.real;
    quat.x = quat_data.i;
    quat.y = quat_data.j;
    quat.z = quat_data.k;
    quat.accuracy_rad = quat_data.rad_accuracy;
    quat.accuracy = convert_accuracy(quat_data.accuracy);
    quat.timestamp_us = now;
    quat.valid = true;

    push_quat(quat);

    // Push euler (derived from quaternion by BNO08x library)
    // Note: get_euler() returns degrees by default
    EulerData euler{};
    euler.roll_deg = euler_data.x;
    euler.pitch_deg = euler_data.y;
    euler.yaw_deg = euler_data.z;
    euler.accuracy_rad = quat_data.rad_accuracy;  // Use quat accuracy (euler's gets corrupted by deg conversion)
    euler.accuracy = convert_accuracy(quat_data.accuracy);
    euler.timestamp_us = now;
    euler.valid = true;

    push_euler(euler);
}

Accuracy BNO08xBackend::convert_accuracy(BNO08xAccuracy acc)
{
    switch (acc) {
        case BNO08xAccuracy::UNRELIABLE:
            return Accuracy::UNRELIABLE;
        case BNO08xAccuracy::LOW:
            return Accuracy::LOW;
        case BNO08xAccuracy::MED:
            return Accuracy::MEDIUM;
        case BNO08xAccuracy::HIGH:
            return Accuracy::HIGH;
        default:
            return Accuracy::UNRELIABLE;
    }
}

} // namespace imu
