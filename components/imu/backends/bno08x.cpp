//
// BNO08x backend implementation
//

#include "bno08x.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"

namespace imu {

static constexpr int MAX_INIT_ATTEMPTS = 5;
static constexpr int RETRY_DELAY_MS = 500;

BNO08xBackend::BNO08xBackend(const Config &cfg) : cfg_(cfg) {}

BNO08xBackend::~BNO08xBackend() {
  stop();
  delete sensor_;
  sensor_ = nullptr;
}

esp_err_t BNO08xBackend::start(const SpiConfig &spi, uint32_t gyro_interval_us,
                               uint32_t accel_interval_us,
                               uint32_t quat_interval_us) {
  if (running_) {

    return ESP_OK;
  }

  bno08x_config_t bno_cfg(spi.host, spi.mosi_pin, spi.miso_pin, spi.sclk_pin,
                          spi.cs_pin, spi.int_pin, spi.rst_pin,
                          spi.clock_speed_hz, cfg_.install_isr_service);

  // Force a clean BNO08x state before init.
  // On warm restart the BNO08x can be stuck mid-transaction with HINT
  // asserted, preventing the NEGEDGE ISR from ever firing.  A manual
  // RST toggle before the library touches the pins fixes this.
  gpio_set_direction(spi.rst_pin, GPIO_MODE_OUTPUT);
  gpio_set_level(spi.rst_pin, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level(spi.rst_pin, 1);
  vTaskDelay(pdMS_TO_TICKS(200)); // let BNO08x complete boot

  bool init_ok = false;
  for (int attempt = 1; attempt <= MAX_INIT_ATTEMPTS; attempt++) {

    sensor_ = new BNO08x(bno_cfg);

    bool result = sensor_->initialize();

    if (result) {
      init_ok = true;
      break;
    }

    delete sensor_;
    sensor_ = nullptr;

    if (attempt < MAX_INIT_ATTEMPTS) {
      int delay = RETRY_DELAY_MS * attempt;

      vTaskDelay(pdMS_TO_TICKS(delay));
    }
  }

  if (!init_ok) {

    return ESP_FAIL;
  }

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

    delete sensor_;
    sensor_ = nullptr;
    return err;
  }

  // Enable accelerometer report
  if (!sensor_->rpt.accelerometer.enable(accel_interval_us)) {

    return ESP_FAIL;
  }
  sensor_->rpt.accelerometer.register_cb([this]() { handle_accel(); });

  // Enable calibrated gyro report
  if (!sensor_->rpt.cal_gyro.enable(gyro_interval_us)) {

    return ESP_FAIL;
  }
  sensor_->rpt.cal_gyro.register_cb([this]() { handle_gyro(); });

  // Enable rotation vector report (mag-referenced for absolute heading)
  if (quat_interval_us > 0) {
    if (!sensor_->rpt.rv.enable(quat_interval_us)) {

      return ESP_FAIL;
    }
    sensor_->rpt.rv.register_cb([this]() { handle_quat(); });

  }

  running_ = true;

  return ESP_OK;
}

esp_err_t BNO08xBackend::stop() {
  if (!running_ || !sensor_) {
    return ESP_OK;
  }

  sensor_->disable_all_reports();
  running_ = false;

  return ESP_OK;
}

void BNO08xBackend::handle_accel() {
  if (!sensor_->rpt.accelerometer.has_new_data()) {
    return;
  }

  bno08x_accel_t data = sensor_->rpt.accelerometer.get();

  int64_t now = esp_timer_get_time();

  AccelData accel{};
  accel.x_ms2 = data.x;
  accel.y_ms2 = data.y;
  accel.z_ms2 = data.z;
  accel.accuracy = convert_accuracy(data.accuracy);
  accel.timestamp_us = now;
  accel.valid = true;

  push_accel(accel);
}

void BNO08xBackend::handle_gyro() {
  if (!sensor_->rpt.cal_gyro.has_new_data()) {
    return;
  }

  bno08x_gyro_t data = sensor_->rpt.cal_gyro.get();
  int64_t now = esp_timer_get_time();

  GyroData gyro{};
  gyro.x_rads = data.x;
  gyro.y_rads = data.y;
  gyro.z_rads = data.z;
  gyro.accuracy = convert_accuracy(data.accuracy);
  gyro.timestamp_us = now;
  gyro.valid = true;

  push_gyro(gyro);
}

void BNO08xBackend::handle_quat() {
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
  euler.accuracy_rad =
      quat_data.rad_accuracy; // Use quat accuracy (euler's gets corrupted by
                              // deg conversion)
  euler.accuracy = convert_accuracy(quat_data.accuracy);
  euler.timestamp_us = now;
  euler.valid = true;

  push_euler(euler);
}

Accuracy BNO08xBackend::convert_accuracy(BNO08xAccuracy acc) {
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
