#include "vehicle_controller.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <rgb_led.h>

#include "blackboard/blackboard.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "imu_types.h"
#include "motor.h"
#include "power_monitor.h"
#include "rclink.h"
#include "soc/gpio_sig_map.h"
#include "soc/spi_periph.h"

namespace vehicle {

Controller::Controller(const Config &config) : config_(config) {}

Controller::~Controller() { stop(); }

esp_err_t Controller::start() {
  BaseType_t result =
      xTaskCreatePinnedToCore(control_task_entry, "vehicle_control", 8192, this,
                              5, &control_task_handle_, 1);

  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create control task");
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

void Controller::stop() {
  if (control_task_handle_ != nullptr) {
    vTaskDelete(control_task_handle_);
    control_task_handle_ = nullptr;
  }
}

void Controller::control_task_entry(void *arg) {
  static_cast<Controller *>(arg)->control_task();
}

[[noreturn]] void Controller::control_task() {
  TickType_t last_wake_time = xTaskGetTickCount();
  constexpr TickType_t frequency = pdMS_TO_TICKS(14);

  while (true) {
    led_update_counter_++;
    if (led_update_counter_ >= 25) {
      update_leds();
      led_update_counter_ = 0;
    }

    if (!rclink::Receiver::instance().valid_data()) {
      handle_rc_loss();
      if (blackboard::latest<imu::GyroData>().valid) {
        countersteer();
      }
      vTaskDelayUntil(&last_wake_time, frequency);
      continue;
    }

    auto channels = rclink::Receiver::instance().get_channels();

    if (channels.ch1.scaled > 25 && !armed_ && channels.ch5.scaled > 25) {
      ESP_LOGI(TAG, "Cannot arm because high thorttle");
      vTaskDelayUntil(&last_wake_time, frequency);
      continue;
    }

    process_rc_input(channels);

    vTaskDelayUntil(&last_wake_time, frequency);
  }
}

void Controller::process_rc_input(const rclink::RcChannels &channels) {
  if (channels.ch1.scaled < 25 && !armed_ && channels.ch5.scaled > 1975) {
    armed_ = true;
    config_.motor_fl.arm();
    config_.motor_fr.arm();
    config_.motor_rl.arm();
    config_.motor_rr.arm();
    ESP_LOGI(TAG, "ARMED");
  } else if (channels.ch5.scaled <= 1975 && armed_) {
    armed_ = false;
    config_.motor_fl.disarm();
    config_.motor_fr.disarm();
    config_.motor_rl.disarm();
    config_.motor_rr.disarm();
    ESP_LOGI(TAG, "DISARMED");
  }

  steering_range_ = channels.ch11.scaled / 20;
  config_.steering_servo.setRange(steering_range_);

  steering_center_ = -(channels.ch4.raw - 992) / 4 + 1500;
  config_.steering_servo.setCenterPoint(steering_center_);

  control_steering(channels.ch2.scaled);

  uint16_t ch7_val = channels.ch7.scaled;
  imu::EulerData euler = blackboard::latest<imu::EulerData>();
  bool imu_ok = euler.valid;

  if (ch7_val >= 1400 && imu_ok) {
    // Mode 1 (HIGH): Active Aero Mode (IMU pitch-stabilized downforce)
    float pitch = euler.pitch_deg;
    constexpr float FRONT_WING_GAIN = 100.0f;
    constexpr float REAR_WING_GAIN = 75.0f;

    // Front wings pitch-stabilized around ch9 baseline
    int32_t front_target = static_cast<int32_t>(channels.ch9.scaled) -
                           static_cast<int32_t>(pitch * FRONT_WING_GAIN);
    rclink::channel_value_t front_pos = static_cast<rclink::channel_value_t>(
        std::clamp<int32_t>(front_target, 0, 2000));
    config_.aux_servo1.setPosition(front_pos);
    config_.aux_servo2.setPosition(front_pos);

    // Rear wing pitch-stabilized around ch10 baseline (lift override removed)
    int32_t rear_target = static_cast<int32_t>(channels.ch10.scaled) -
                          static_cast<int32_t>(pitch * REAR_WING_GAIN);
    rclink::channel_value_t rear_pos = static_cast<rclink::channel_value_t>(
        std::clamp<int32_t>(rear_target, 0, 2000));
    config_.aero_servo.setPosition(rear_pos);
  } else if (ch7_val >= 600 && ch7_val < 1400) {
    // Mode 2 (MID): Aileron/Roll Mode (differential front wings based on steering input & IMU roll)
    constexpr float AILERON_GAIN = 0.5f;
    int32_t steer_diff = static_cast<int32_t>((static_cast<int32_t>(channels.ch2.scaled) - 1000) * AILERON_GAIN);

    int32_t roll_diff = 0;
    if (imu_ok) {
      float roll = euler.roll_deg;
      constexpr float AILERON_ROLL_GAIN = 50.0f;
      roll_diff = static_cast<int32_t>(roll * AILERON_ROLL_GAIN);
    }

    int32_t total_diff = steer_diff + roll_diff;

    int32_t aux1_target = static_cast<int32_t>(channels.ch9.scaled) - total_diff;
    int32_t aux2_target = static_cast<int32_t>(channels.ch9.scaled) + total_diff;

    config_.aux_servo1.setPosition(static_cast<rclink::channel_value_t>(
        std::clamp<int32_t>(aux1_target, 0, 2000)));
    config_.aux_servo2.setPosition(static_cast<rclink::channel_value_t>(
        std::clamp<int32_t>(aux2_target, 0, 2000)));

    // Rear wing remains in manual control (ch10)
    config_.aero_servo.setPosition(channels.ch10.scaled);
  } else {
    // Mode 3 (LOW or fallback): Manual Mode
    config_.aux_servo1.setPosition(channels.ch9.scaled);
    config_.aux_servo2.setPosition(channels.ch9.scaled);
    config_.aero_servo.setPosition(channels.ch10.scaled);
  }

  if (armed_) {
    control_motors(channels);
  }
}

void Controller::control_steering(rclink::channel_value_t steering_input) {
  config_.steering_servo.setPosition(steering_input);
}

void Controller::control_motors(const rclink::RcChannels &channels) {
  motor::Motor *motors[] = {&config_.motor_fr, &config_.motor_fl,
                            &config_.motor_rl, &config_.motor_rr};

  motor::RpmSample rpm[4];
  for (int i = 0; i < 4; i++) {
    rpm[i] = motors[i]->rpm();
  }

  const rclink::channel_value_t throttle_input = channels.ch1.scaled;
  const int16_t command = throttle_input < THROTTLE_DEADBAND
                              ? 0
                              : static_cast<int16_t>(
                                    (throttle_input / 2000.0f) * motor::HALF_SPAN);

  for (int i = 0; i < 4; i++) {
    motors[i]->command(command);
  }

  blackboard::publish(motor::RpmReading{
      .rpm = {rpm[0].rpm, rpm[1].rpm, rpm[2].rpm, rpm[3].rpm},
      .status = {static_cast<uint8_t>(rpm[0].status),
                 static_cast<uint8_t>(rpm[1].status),
                 static_cast<uint8_t>(rpm[2].status),
                 static_cast<uint8_t>(rpm[3].status)},
      .count = 4});
}

void Controller::countersteer() {
  constexpr float GAIN = 300.0f;
  float yaw_rate = blackboard::latest<imu::GyroData>().z_rads;
  int position = 1000 + static_cast<int>(GAIN * yaw_rate);
  config_.steering_servo.setPosition(
      static_cast<rclink::channel_value_t>(std::clamp(position, 0, 2000)));
}

void Controller::handle_rc_loss() {
  armed_ = false;
  config_.motor_fr.disarm();
  config_.motor_fl.disarm();
  config_.motor_rl.disarm();
  config_.motor_rr.disarm();
  config_.steering_servo.setFailsafe();
  config_.aero_servo.setFailsafe();
  config_.aux_servo1.setFailsafe();
  config_.aux_servo2.setFailsafe();
}

void Controller::refresh_headlights(uint8_t r, uint8_t g, uint8_t b) {
  float mult = get_brightness_multiplier();
  uint8_t br = std::min(255, (int)(r * mult));
  uint8_t bg = std::min(255, (int)(g * mult));
  uint8_t bb = std::min(255, (int)(b * mult));

  uint8_t pixels[14][3];
  for (int i = 0; i < 14; i++) {
    pixels[i][0] = br;
    pixels[i][1] = bg;
    pixels[i][2] = bb;
  }
  // Refresh GPIO 5 using the central Rgb class
  led::Rgb::instance().refresh_strip_gpio(5, 14, pixels);
}

void Controller::refresh_rear_strip(const uint8_t pixels[34][3]) {
  float mult = get_brightness_multiplier();
  uint8_t scaled_pixels[34][3];
  for (int i = 0; i < 34; i++) {
    scaled_pixels[i][0] = std::min(255, (int)(pixels[i][0] * mult));
    scaled_pixels[i][1] = std::min(255, (int)(pixels[i][1] * mult));
    scaled_pixels[i][2] = std::min(255, (int)(pixels[i][2] * mult));
  }
  // Refresh GPIO 21 using the central Rgb class
  led::Rgb::instance().refresh_strip_gpio(21, 34, scaled_pixels);
}

float Controller::get_brightness_multiplier() {
  if (!rclink::Receiver::instance().valid_data()) {
    return 1.0f;
  }
  uint16_t ch6_val = rclink::Receiver::instance().get_channels().ch6.scaled;
  if (ch6_val < 600) {
    return 0.0f;
  } else if (ch6_val < 1400) {
    return 1.0f;
  } else {
    return 2.125f;
  }
}

void Controller::update_leds() {
  // 1. Check cell voltage override
  PowerMonitor::Reading power = blackboard::latest<PowerMonitor::Reading>();
  int32_t cell_voltage_mv =
      power.cell_count > 0 ? power.voltage_mv / power.cell_count : 0;

  bool low_voltage = false;
  // Only verify low voltage if we have a valid battery reading
  if (power.valid && power.cell_count > 0 && cell_voltage_mv < 3400) {
    low_voltage = true;
  }

  if (low_voltage) {
    // Flash all lamps RED continuously (toggle every 250ms / 5 loops * 50ms)
    red_flash_counter_++;
    if (red_flash_counter_ >= 5) {
      red_flash_state_ = !red_flash_state_;
      red_flash_counter_ = 0;
    }

    uint8_t r = red_flash_state_ ? 150 : 0;

    // Flash headlights
    refresh_headlights(r, 0, 0);

    // Flash rear strip
    uint8_t rear_pixels[34][3] = {0};
    if (red_flash_state_) {
      for (int i = 0; i < 34; i++) {
        rear_pixels[i][0] = 150;
      }
    }
    refresh_rear_strip(rear_pixels);
    return;
  }

  // 2. Normal operation: transition based on armed state
  // --- Front Headlights (GPIO 8) ---
  // Orange/Amber (disarmed): R=120, G=42, B=0
  // White (armed): R=120, G=120, B=120
  uint8_t target_r = 120;
  uint8_t target_g = armed_ ? 120 : 42;
  uint8_t target_b = armed_ ? 120 : 0;

  // Transition headlights color
  if (current_led_r_ < target_r)
    current_led_r_ = std::min((int)target_r, current_led_r_ + 5);
  else if (current_led_r_ > target_r)
    current_led_r_ = std::max((int)target_r, current_led_r_ - 5);

  if (current_led_g_ < target_g)
    current_led_g_ = std::min((int)target_g, current_led_g_ + 5);
  else if (current_led_g_ > target_g)
    current_led_g_ = std::max((int)target_g, current_led_g_ - 5);

  if (current_led_b_ < target_b)
    current_led_b_ = std::min((int)target_b, current_led_b_ + 5);
  else if (current_led_b_ > target_b)
    current_led_b_ = std::max((int)target_b, current_led_b_ - 5);

  refresh_headlights(current_led_r_, current_led_g_, current_led_b_);

  // --- Rear Strip (GPIO 21) ---
  uint8_t rear_pixels[34][3] = {0};

  if (armed_) {
    // Track transition from disarmed to armed
    if (!last_armed_state_) {
      // State transition from disarmed to armed: trigger crossing animation
      arming_animation_active_ = true;
      arming_animation_step_ = 0;
      last_armed_state_ = true;
      // Clear trails so we start clean
      for (int i = 0; i < 34; i++) {
        seeker_trails_[i] = 0.0f;
      }
    }

    if (arming_animation_active_) {
      // Phase 1: Double Crossing (steps 0 to 33)
      if (arming_animation_step_ < 34) {
        // Decay trails
        for (int i = 0; i < 34; i++) {
          seeker_trails_[i] *= 0.82f;
        }

        int pos1 = arming_animation_step_;
        int pos2 = 33 - arming_animation_step_;
        seeker_trails_[pos1] = 1.0f;
        seeker_trails_[pos2] = 1.0f;

        // Set soft neighbors for a wider scan point
        if (pos1 + 1 < 34)
          seeker_trails_[pos1 + 1] = std::max(seeker_trails_[pos1 + 1], 0.5f);
        if (pos1 - 1 >= 0)
          seeker_trails_[pos1 - 1] = std::max(seeker_trails_[pos1 - 1], 0.5f);
        if (pos2 + 1 < 34)
          seeker_trails_[pos2 + 1] = std::max(seeker_trails_[pos2 + 1], 0.5f);
        if (pos2 - 1 >= 0)
          seeker_trails_[pos2 - 1] = std::max(seeker_trails_[pos2 - 1], 0.5f);

        for (int i = 0; i < 34; i++) {
          rear_pixels[i][0] = (uint8_t)(seeker_trails_[i] * 120.0f);
        }
        arming_animation_step_++;
      }
      // Phase 2: Inward Fill (steps 34 to 50)
      else if (arming_animation_step_ < 51) {
        // Decay trails for the non-filled part
        for (int i = 0; i < 34; i++) {
          seeker_trails_[i] *= 0.82f;
        }

        int fill_step = arming_animation_step_ - 34;
        for (int j = 0; j <= fill_step; j++) {
          seeker_trails_[j] = 1.0f;
          seeker_trails_[33 - j] = 1.0f;
        }

        for (int i = 0; i < 34; i++) {
          rear_pixels[i][0] = (uint8_t)(seeker_trails_[i] * 120.0f);
        }
        arming_animation_step_++;
      }
      // Phase 3: Animation complete
      else {
        arming_animation_active_ = false;
        for (int i = 0; i < 34; i++) {
          rear_pixels[i][0] = 120; // Full solid red
        }
      }
    } else {
      // Normal armed state: Solid Red
      for (int i = 0; i < 34; i++) {
        rear_pixels[i][0] = 120;
      }
    }
  } else {
    // Disarmed state: Seeker animation (Red)
    if (last_armed_state_) {
      // Just disarmed: abort any arming animation and reset seeker
      arming_animation_active_ = false;
      last_armed_state_ = false;
      // Clear trails
      for (int i = 0; i < 34; i++) {
        seeker_trails_[i] = 0.0f;
      }
    }

    // Seeker logic: decay trails
    for (int i = 0; i < 34; i++) {
      seeker_trails_[i] *= 0.82f;
    }

    // Move seeker position
    seeker_pos_ += seeker_dir_;
    if (seeker_pos_ >= 33) {
      seeker_pos_ = 33;
      seeker_dir_ = -1;
    } else if (seeker_pos_ <= 0) {
      seeker_pos_ = 0;
      seeker_dir_ = 1;
    }

    // Light up the seeker position and neighbors
    seeker_trails_[seeker_pos_] = 1.0f;
    if (seeker_pos_ + 1 < 34) {
      seeker_trails_[seeker_pos_ + 1] =
          std::max(seeker_trails_[seeker_pos_ + 1], 0.5f);
    }
    if (seeker_pos_ - 1 >= 0) {
      seeker_trails_[seeker_pos_ - 1] =
          std::max(seeker_trails_[seeker_pos_ - 1], 0.5f);
    }

    // Fill rear pixels based on seeker trails (Red color)
    for (int i = 0; i < 34; i++) {
      rear_pixels[i][0] = (uint8_t)(seeker_trails_[i] * 120.0f);
      rear_pixels[i][1] = 0;
      rear_pixels[i][2] = 0;
    }
  }

  refresh_rear_strip(rear_pixels);
}

} // namespace vehicle
