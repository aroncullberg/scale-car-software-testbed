//
// BNO08x backend implementation
//

#include "bno08x.h"
#include "imu.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace imu
{

static const char* TAG = "imu-bno08x";

static constexpr int MAX_INIT_ATTEMPTS = 5;
static constexpr int RETRY_DELAY_MS = 500;

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

    // Force a clean BNO08x state before init.
    // On warm restart the BNO08x can be stuck mid-transaction with HINT
    // asserted, preventing the NEGEDGE ISR from ever firing.  A manual
    // RST toggle before the library touches the pins fixes this.
    gpio_set_direction(spi.rst_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(spi.rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(spi.rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(200));   // let BNO08x complete boot

    gpio_num_t hint_pin = spi.int_pin;
    ESP_LOGI(TAG, "HINT pin (%d) state after pre-reset: %d", hint_pin, gpio_get_level(hint_pin));

    bool init_ok = false;
    for (int attempt = 1; attempt <= MAX_INIT_ATTEMPTS; attempt++) {
        ESP_LOGI(TAG, "init attempt %d/%d", attempt, MAX_INIT_ATTEMPTS);

        sensor_ = new BNO08x(bno_cfg);

        bool result = sensor_->initialize();

        ESP_LOGI(TAG, "initialize() returned %s, HINT pin: %d",
            result ? "OK" : "FAIL", gpio_get_level(hint_pin));

        if (result) {
            init_ok = true;
            break;
        }

        delete sensor_;
        sensor_ = nullptr;

        if (attempt < MAX_INIT_ATTEMPTS) {
            int delay = RETRY_DELAY_MS * attempt;
            ESP_LOGW(TAG, "retrying in %d ms...", delay);
            vTaskDelay(pdMS_TO_TICKS(delay));
        }
    }

    if (!init_ok) {
        ESP_LOGE(TAG, "failed to initialize BNO08x after %d attempts", MAX_INIT_ATTEMPTS);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BNO08x initialized successfully");

    // Let the library's sh2_HAL_service_task (priority 7) drain all
    // post-boot HINT assertions before we try to enable reports.
    // Without this delay, rpt.enable() → shtp txProcess() can enter
    // an infinite retry loop because HINT is not asserted when it
    // tries to write (and spi_wait_for_int timeouts trigger
    // destructive hardware resets).
    vTaskDelay(pdMS_TO_TICKS(300));

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

    int64_t now = esp_timer_get_time();

    // Log first sample and periodic heartbeat
    if (accel_count_ == 0) {
        ESP_LOGI(TAG, "first accel sample: x=%.2f y=%.2f z=%.2f", data.x, data.y, data.z);
    }
    accel_count_++;
    if (accel_count_ % 1000 == 0) {
        ESP_LOGI(TAG, "accel heartbeat: %lu samples, last dt=%lld us",
            accel_count_, now - last_accel_us_);
    }
    last_accel_us_ = now;

    AccelData accel{};
    accel.x_ms2 = data.x;
    accel.y_ms2 = data.y;
    accel.z_ms2 = data.z;
    accel.accuracy = convert_accuracy(data.accuracy);
    accel.timestamp_us = now;
    accel.valid = true;

    push_accel(accel);
}

void BNO08xBackend::handle_gyro()
{
    if (!sensor_->rpt.cal_gyro.has_new_data()) {
        return;
    }

    bno08x_gyro_t data = sensor_->rpt.cal_gyro.get();
    int64_t now = esp_timer_get_time();

    if (gyro_count_ == 0) {
        ESP_LOGI(TAG, "first gyro sample: x=%.2f y=%.2f z=%.2f", data.x, data.y, data.z);
    }
    gyro_count_++;
    if (gyro_count_ % 1000 == 0) {
        ESP_LOGI(TAG, "gyro heartbeat: %lu samples, last dt=%lld us",
            gyro_count_, now - last_gyro_us_);
    }
    last_gyro_us_ = now;

    GyroData gyro{};
    gyro.x_rads = data.x;
    gyro.y_rads = data.y;
    gyro.z_rads = data.z;
    gyro.accuracy = convert_accuracy(data.accuracy);
    gyro.timestamp_us = now;
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

    if (quat_count_ == 0) {
        ESP_LOGI(TAG, "first quat sample: w=%.3f x=%.3f y=%.3f z=%.3f",
            quat_data.real, quat_data.i, quat_data.j, quat_data.k);
    }
    quat_count_++;
    if (quat_count_ % 200 == 0) {
        ESP_LOGI(TAG, "quat heartbeat: %lu samples, last dt=%lld us",
            quat_count_, (int64_t)now - last_quat_us_);
    }
    last_quat_us_ = now;

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
