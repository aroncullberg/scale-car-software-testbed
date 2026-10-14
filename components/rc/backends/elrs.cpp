//
// Created by cullb on 2025-04-26.
//

#include "elrs.h"
#include "rclink.h"
#include "blackboard/blackboard.h"

#include "imu_types.h"
#include "gps.h"
#include "motor.h"
#include "power_monitor.h"

extern "C" {
#include "ESP_CRSF.h"
}

using rclink::channel_value_t;
using rclink::ChannelIndex;

#undef TAG
#define TAG "ExpressLRS"

namespace proto {
ExpressLRS::ExpressLRS(const Config &cfg) : cfg_(cfg) {};

ExpressLRS::~ExpressLRS() {
  ExpressLRS::stop();
  ESP_LOGI(TAG, "ExpressLRS instance destroyed");
}

esp_err_t ExpressLRS::start(const rclink::uart_pins_t &pins,
                            uint32_t timeout_ms) {
  ESP_LOGD(TAG, "Starting ELRS on UART%d (RX: %d, TX: %d)", pins.uart_num,
           pins.rx_pin, pins.tx_pin);

  if (running_) {
    return ESP_ERR_INVALID_STATE;
  }

  // Update config with provided pins
  cfg_.uart_num = pins.uart_num;
  cfg_.uart_rx_pin = pins.rx_pin;
  cfg_.uart_tx_pin = pins.tx_pin;

  // Don't suppress CRSF logging during detection - we need debug visibility
  esp_log_level_set("uart", ESP_LOG_WARN);
  // esp_log_level_set("CRSF", ESP_LOG_WARN);  // Keep CRSF logs visible for
  // debugging

  // Initialize CRSF with our pins
  crsf_config_t config = {.uart_num = static_cast<uint8_t>(cfg_.uart_num),
                          .tx_pin = static_cast<uint8_t>(cfg_.uart_tx_pin),
                          .rx_pin = static_cast<uint8_t>(cfg_.uart_rx_pin)};

  esp_err_t ret = CRSF_init(&config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CRSF_init failed: %s", esp_err_to_name(ret));
    return ESP_FAIL;
  }

  // Try to detect valid CRSF frames within timeout
  TickType_t start_time = xTaskGetTickCount();
  TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  crsf_channels_t channels = {};
  uint32_t valid_frames = 0;
  const uint32_t required_frames = 2;

  ESP_LOGD(TAG, "Detecting ELRS/CRSF frames for %lu ms...", timeout_ms);

  while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
    if (CRSF_receive_channels(&channels) == ESP_OK) {
      if (channels.ch1 > 0 && channels.ch1 < 2048) {
        valid_frames++;
        ESP_LOGI(TAG,
                 "✓ Valid CRSF frame %lu (ch1: %d, ch2: %d, ch3: %d, ch4: %d)",
                 valid_frames, channels.ch1, channels.ch2, channels.ch3,
                 channels.ch4);

        if (valid_frames >= required_frames) {
          ESP_LOGI(TAG, "✓ ELRS protocol detected! Starting main task...");

          // Start the ExpressLRS task for continuous operation
          BaseType_t res = xTaskCreatePinnedToCore(taskEntry, "ExpressLRS task",
                                                   4096, this, 5, &task_, 1);

          running_ = (res == pdPASS);
          if (running_) {
            ESP_LOGI(TAG, "✓ ExpressLRS backend task started successfully");
          } else {
            ESP_LOGE(TAG, "✗ Failed to start ExpressLRS backend task");
          }
          return running_ ? ESP_OK : ESP_FAIL;
        }
      } else {
        ESP_LOGD(TAG, "Frame rejected: ch1=%d out of range", channels.ch1);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  ESP_LOGD(TAG, "ELRS detection timeout (%lu valid frames, needed %lu)",
           valid_frames, required_frames);
  CRSF_cleanup();
  return ESP_ERR_TIMEOUT;
}

esp_err_t ExpressLRS::stop() {
  if (!running_)
    return ESP_ERR_INVALID_STATE;
  vTaskDelete(task_);
  task_ = nullptr;
  running_ = false;
  return ESP_OK;
}

void ExpressLRS::taskEntry(void *arg) {
  auto *self = static_cast<ExpressLRS *>(arg);
  self->run();
}

void ExpressLRS::run() {
  crsf_channels_t channels = {};
  TickType_t last_wake_time = xTaskGetTickCount();
  bool was_valid = false;
  uint32_t valid_frames = 0;
  uint32_t invalid_frames = 0;

  while (true) {
    esp_err_t result = CRSF_receive_channels(&channels);

    if (result == ESP_OK) {
      valid_frames++;

      if (!was_valid) {
        ESP_LOGI(TAG, "✓ RC link established (valid=%lu, invalid=%lu)",
                 valid_frames, invalid_frames);
        was_valid = true;
      }

      set_valid_data(true);

      // Push all 16 channels
      push(ChannelIndex::CH1, channels.ch1, rawToScaled(channels.ch1));
      push(ChannelIndex::CH2, channels.ch2, rawToScaled(channels.ch2));
      push(ChannelIndex::CH3, channels.ch3, rawToScaled(channels.ch3));
      push(ChannelIndex::CH4, channels.ch4, rawToScaled(channels.ch4));
      push(ChannelIndex::CH5, channels.ch5, rawToScaled(channels.ch5));
      push(ChannelIndex::CH6, channels.ch6, rawToScaled(channels.ch6));
      push(ChannelIndex::CH7, channels.ch7, rawToScaled(channels.ch7));
      push(ChannelIndex::CH8, channels.ch8, rawToScaled(channels.ch8));
      push(ChannelIndex::CH9, channels.ch9, rawToScaled(channels.ch9));
      push(ChannelIndex::CH10, channels.ch10, rawToScaled(channels.ch10));
      push(ChannelIndex::CH11, channels.ch11, rawToScaled(channels.ch11));
      push(ChannelIndex::CH12, channels.ch12, rawToScaled(channels.ch12));
      push(ChannelIndex::CH13, channels.ch13, rawToScaled(channels.ch13));
      push(ChannelIndex::CH14, channels.ch14, rawToScaled(channels.ch14));
      push(ChannelIndex::CH15, channels.ch15, rawToScaled(channels.ch15));
      push(ChannelIndex::CH16, channels.ch16, rawToScaled(channels.ch16));

      blackboard::publish(rclink::Receiver::instance().get_channels());

      // Log channel values periodically (every 100 valid frames)
      if (valid_frames % 100 == 0) {
        ESP_LOGD(TAG, "Channels: ch1=%d ch2=%d ch3=%d ch4=%d", channels.ch1,
                 channels.ch2, channels.ch3, channels.ch4);
      }

      // Read latest blackboard values and send to RC link (~100ms interval)
      if (valid_frames % 10 == 0) {
        send_telemetry();
      }
    } else if (result == ESP_ERR_TIMEOUT) {
      // Link lost - data is stale
      invalid_frames++;

      if (was_valid) {
        ESP_LOGW(TAG, "✗ RC link lost (valid=%lu, invalid=%lu)", valid_frames,
                 invalid_frames);
        was_valid = false;
      }

      set_valid_data(false);
    } else {
      // Other error (shouldn't happen in normal operation)
      ESP_LOGE(TAG, "CRSF_receive_channels error: %s", esp_err_to_name(result));
    }

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
  }
}

void ExpressLRS::send_telemetry() {
  if (!running_) {
    return;
  }

  auto battery = blackboard::latest<PowerMonitor::Reading>();
  if (battery.valid) {
    crsf_battery_t crsf_battery = {
        .voltage = static_cast<uint16_t>(battery.voltage_mv / 100), // 0.1V units
        .current = static_cast<uint16_t>(battery.current_ma / 100), // 0.1A units
        .capacity = battery.consumed_mah,
        .remaining = battery.remaining_pct,
    };
    CRSF_send_battery_data(CRSF_DEST_FC, &crsf_battery);
  }

  auto location = nav::Gps::instance().getLocation();
  if (location.isValid) {
    crsf_gps_t crsf_gps = {
        .latitude = location.lat_e7,
        .longitude = location.lon_e7,
        .groundspeed = static_cast<uint16_t>(nav::Gps::instance().getSpeed().speed_kmph),
        .heading = static_cast<uint16_t>(nav::Gps::instance().getCourse().course_cd / 100),
        .altitude = static_cast<uint16_t>(nav::Gps::instance().getAltitude().altitude_m),
        .satellites = static_cast<uint8_t>(nav::Gps::instance().getSatellite().satellites),
    };
    CRSF_send_gps_data(CRSF_DEST_FC, &crsf_gps);
  }

  auto euler = blackboard::latest<imu::EulerData>();
  if (euler.valid) {
    // Float degrees to int16 with LSB = 100 µrad.
    constexpr float deg_to_crsf = 174.5329f;
    crsf_attitude_t crsf_attitude = {
        .pitch = static_cast<int16_t>(euler.pitch_deg * deg_to_crsf),
        .roll = static_cast<int16_t>(euler.roll_deg * deg_to_crsf),
        .yaw = static_cast<int16_t>(euler.yaw_deg * deg_to_crsf),
    };
    CRSF_send_attitude_data(CRSF_DEST_FC, &crsf_attitude);
  }

  auto rpm = blackboard::latest<motor::RpmReading>();
  if (rpm.count > 0) {
    CRSF_send_rpm_data(CRSF_DEST_FC, 0, rpm.rpm, rpm.count);
  }

  auto accel = blackboard::latest<imu::AccelData>();
  auto gyro = blackboard::latest<imu::GyroData>();
  if (accel.valid && gyro.valid) {
    // CRSF gyro: LSB = INT16_MAX / 2000 DPS. Input rad/s → deg/s * scale.
    constexpr float rads_to_crsf_gyro =
        (180.0f / 3.14159265f) * (32767.0f / 2000.0f);
    // CRSF accel: LSB = INT16_MAX / 16 G. Input m/s² → G * scale.
    constexpr float ms2_to_crsf_accel = (1.0f / 9.80665f) * (32767.0f / 16.0f);
    crsf_accelgyro_t crsf_ag = {
        .sample_time = static_cast<uint32_t>(accel.timestamp_us),
        .gyro_x = static_cast<int16_t>(gyro.x_rads * rads_to_crsf_gyro),
        .gyro_y = static_cast<int16_t>(gyro.y_rads * rads_to_crsf_gyro),
        .gyro_z = static_cast<int16_t>(gyro.z_rads * rads_to_crsf_gyro),
        .acc_x = static_cast<int16_t>(accel.x_ms2 * ms2_to_crsf_accel),
        .acc_y = static_cast<int16_t>(accel.y_ms2 * ms2_to_crsf_accel),
        .acc_z = static_cast<int16_t>(accel.z_ms2 * ms2_to_crsf_accel),
        .gyro_temp = 0,
    };
    CRSF_send_accelgyro_data(CRSF_DEST_FC, &crsf_ag);
  }
}

} // namespace proto
