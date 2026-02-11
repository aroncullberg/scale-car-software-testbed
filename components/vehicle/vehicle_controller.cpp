#include "vehicle_controller.h"

#include <algorithm>
#include <cmath>
#include <rgb_led.h>

#include "esp_log.h"
#include "rclink.h"
#include "telemetry.h"
#include "imu.h"

namespace vehicle {

Controller::Controller(const Config& config) : config_(config) {}

Controller::~Controller() {
    stop();
}

esp_err_t Controller::start() {
    BaseType_t result = xTaskCreatePinnedToCore(
        control_task_entry,
        "vehicle_control",
        8192,
        this,
        5,
        &control_task_handle_,
        1
    );

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

void Controller::control_task_entry(void* arg) {
    static_cast<Controller*>(arg)->control_task();
}

[[noreturn]] void Controller::control_task() {
    TickType_t last_wake_time = xTaskGetTickCount();
    constexpr TickType_t frequency = pdMS_TO_TICKS(2);

    while (true) {
        if (!rclink::Receiver::instance().valid_data()) {
            handle_rc_loss();
            vTaskDelayUntil(&last_wake_time, frequency);
            continue;
        }

        auto channels = rclink::Receiver::instance().get_channels();

        if (channels.ch1.scaled > 25 && !armed_) {
            vTaskDelayUntil(&last_wake_time, frequency);
            continue;
        }

        process_rc_input(channels);

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

void Controller::process_rc_input(const rclink::RcChannels& channels) {
    bool diff_on = channels.ch6.scaled > 1975;

    if (channels.ch1.scaled < 25 && !armed_ && channels.ch5.scaled > 1975) {
        armed_ = true;
        ESP_LOGI(TAG, "ARMED");
        led::Rgb::instance().set_color(diff_on ? 128 : 0, 255, 0);
    } else if (channels.ch5.scaled <= 1975 && armed_) {
        armed_ = false;
        ESP_LOGI(TAG, "DISARMED");
        led::Rgb::instance().set_color(0, 0, 255);
    }

    if (armed_ && diff_on != diff_on_) {
        diff_on_ = diff_on;
        led::Rgb::instance().set_color(diff_on ? 128 : 0, 255, 0);
    }

    steering_range_ = channels.ch3.scaled / 20;
    config_.steering_servo.setRange(steering_range_);

    steering_center_ = -(channels.ch4.raw - 992) / 4 + 1500;
    config_.steering_servo.setCenterPoint(steering_center_);

    reverse_ = channels.ch1.scaled < 200 ? channels.ch8.scaled > 1975 : reverse_;

    if (armed_) {
        control_motors(channels, channels.ch1.scaled, reverse_);
        control_steering(channels.ch2.scaled);
    } else if (imu::IMU::instance().has_valid_gyro()) {
        countersteer();
    } else {
        control_steering(channels.ch2.scaled);
    }
}

void Controller::control_steering(rclink::channel_value_t steering_input) {
    config_.steering_servo.setPosition(steering_input);
}

void Controller::control_motors(const rclink::RcChannels& channels, rclink::channel_value_t throttle_input, bool reverse) {
    int16_t dshot_value = reverse
        ? 48 + throttle_input / 2
        : 1048 + throttle_input / 2;

    dshot_value = std::clamp(dshot_value, static_cast<int16_t>(DSHOT_THROTTLE_MIN),
                                          static_cast<int16_t>(DSHOT_THROTTLE_MAX));

    int16_t motor_dshot[] = { dshot_value, dshot_value, dshot_value, dshot_value };

    // Software diff: ch6=enable, ch7=steer mix, ch9=throttle mix. Forward only.
    int16_t boost = 0;
    if (!reverse && channels.ch6.scaled > 1975) {
        float steer_factor = (static_cast<int16_t>(channels.ch2.scaled) - 1000) / 1000.0f;
        float steer_mix = channels.ch7.scaled / 2000.0f;
        float throttle_mix = channels.ch9.scaled / 2000.0f;
        boost = static_cast<int16_t>(std::abs(steer_factor) * steer_mix
                                     * std::pow(throttle_input / 2000.0f, 1.0f + throttle_mix * 2.0f)
                                     * 1000.0f);

        if (steer_factor > 0.0f) {
            motor_dshot[1] = std::min(static_cast<int16_t>(dshot_value + boost), static_cast<int16_t>(DSHOT_THROTTLE_MAX));
            motor_dshot[2] = std::min(static_cast<int16_t>(dshot_value + boost), static_cast<int16_t>(DSHOT_THROTTLE_MAX));
        } else {
            motor_dshot[0] = std::min(static_cast<int16_t>(dshot_value + boost), static_cast<int16_t>(DSHOT_THROTTLE_MAX));
            motor_dshot[3] = std::min(static_cast<int16_t>(dshot_value + boost), static_cast<int16_t>(DSHOT_THROTTLE_MAX));
        }
    }

    ESP_LOGI(TAG, "thr=%u ch6=%u ch7=%u ch9=%u steer=%u | boost=%d dshot FR=%d FL=%d RL=%d RR=%d",
             throttle_input, channels.ch6.scaled, channels.ch7.scaled, channels.ch9.scaled,
             channels.ch2.scaled, boost, motor_dshot[0], motor_dshot[1], motor_dshot[2], motor_dshot[3]);

    const bool enabled[] = {
        channels.ch13.scaled > 1975,
        channels.ch14.scaled > 1975,
        channels.ch15.scaled > 1975,
        channels.ch16.scaled > 1975,
    };

    DShotRMT* motors[] = {
        &config_.motor_fr, &config_.motor_fl,
        &config_.motor_rl, &config_.motor_rr
    };

    for (int i = 0; i < 4; i++) {
        motors[i]->sendThrottle(enabled[i] ? motor_dshot[i] : 1048);
        uint32_t erpm = motors[i]->getErpm();
        motor_erpm_[i] = erpm != 65535 ? erpm : 0;
    }

    float erpm_to_rpm = DShotRMT::getErpmToRpmRatio(config_.motor_poles);
    int32_t rpm[4];
    for (int i = 0; i < 4; i++)
        rpm[i] = static_cast<int32_t>(motor_erpm_[i] * erpm_to_rpm);

    // ESP_LOGI(TAG, "RPM: FL=%ld FR=%ld RL=%ld RR=%ld", rpm[0], rpm[1], rpm[2], rpm[3]);

    telemetry::publish(telemetry::RpmTelemetry{
        .source_id = 0,
        .rpm_values = { rpm[0], rpm[1], rpm[2], rpm[3] },
        .count = 4
    });
}

void Controller::countersteer() {
    constexpr float GAIN = 300.0f;
    float yaw_rate = imu::IMU::instance().get_gyro().z_rads;
    int position = 1000 + static_cast<int>(GAIN * yaw_rate);
    config_.steering_servo.setPosition(
        static_cast<rclink::channel_value_t>(std::clamp(position, 0, 2000)));
}

void Controller::handle_rc_loss() {
    armed_ = false;
    config_.steering_servo.setFailsafe();
}

} // namespace vehicle
