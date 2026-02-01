#include "vehicle_controller.h"

#include <algorithm>
#include <cmath>

#include "esp_log.h"
#include "rclink.h"
#include "gps.h"
// #include "imu.h"

namespace vehicle {

Controller::Controller(const Config& config) : config_(config) {
}

Controller::~Controller() {
    stop();
}

esp_err_t Controller::init() {
    ESP_LOGI(TAG, "Initializing vehicle controller");

    // Initialize motor states
    for (auto& state : motor_states_) {
        state = MotorState{};
    }

    ESP_LOGI(TAG, "Vehicle controller initialized");
    return ESP_OK;
}

esp_err_t Controller::start() {
    ESP_LOGI(TAG, "Starting vehicle controller");

    // Create control loop task
    BaseType_t result = xTaskCreatePinnedToCore(
        control_task_entry,
        "vehicle_control",
        8192,                    // Stack size
        this,                    // Pass this pointer
        5,                       // Priority
        &control_task_handle_,
        1                        // Pin to core 1
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Vehicle controller started");
    return ESP_OK;
}

void Controller::stop() {
    if (control_task_handle_ != nullptr) {
        vTaskDelete(control_task_handle_);
        control_task_handle_ = nullptr;
        ESP_LOGI(TAG, "Vehicle controller stopped");
    }
}

void Controller::control_task_entry(void* arg) {
    auto* controller = static_cast<Controller*>(arg);
    controller->control_task();
}

[[noreturn]] void Controller::control_task() {
    ESP_LOGI(TAG, "Vehicle control task started (100Hz)");

    TickType_t last_wake_time = xTaskGetTickCount();
    constexpr TickType_t frequency = pdMS_TO_TICKS(2); // 100Hz = 10ms

    while (true) {
        control_cycle_count_++;

        // Check RC data validity
        if (!rclink::Receiver::instance().valid_data()) {
            ESP_LOGI(TAG, "receiver timeout");
            handle_rc_loss();
            vTaskDelayUntil(&last_wake_time, frequency);
            continue;
        }

        // Get RC channels
        auto channels = rclink::Receiver::instance().get_channels();

        if (channels.ch1.scaled > 25 && !armed_) {
            ESP_LOGI(TAG, "Requirements for armning not fullfilled");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
            // dont allow control loop to start if throttle not 0
        }

        // Process input and control outputs
        process_rc_input(channels);

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

void Controller::process_rc_input(const rclink::RcChannels& channels) {
    // Channel mapping from reference code
    constexpr auto ch_steeringrange = 2; // CH4
    constexpr auto ch_steeroffset = 3;        // CH5 (not CH6, that's used for reverse)

    // Access channels by index (convert to array for convenience)
    const rclink::ChannelPair* ch_array = &channels.ch1;

    // Arming logic (LED now managed by RC component based on CH2)
    if (channels.ch1.scaled < 25 && !armed_ && channels.ch5.scaled > 1975) {
        armed_ = true;
        ESP_LOGI(TAG, "ARMED");
    } else if (channels.ch5.scaled <= 1975 && armed_) {
        armed_ = false;
        ESP_LOGI(TAG, "DISARMED");
    }

    steering_range_ = ch_array[ch_steeringrange].scaled / 20;
    config_.steering_servo.setRange(steering_range_);

    steering_center_ = (ch_array[ch_steeroffset].raw - 992) / 2 + 1500;
    config_.steering_servo.setCenterPoint(steering_center_);

    reverse_ = channels.ch1.scaled < 25 ? channels.ch10.scaled > 1975 : reverse_;

    if (armed_) {
        control_motors(channels.ch1.scaled, !reverse_);
    }

    control_steering(channels.ch2.scaled);
}

void Controller::control_steering(rclink::channel_value_t steering_input) {
    // ESP_LOGI(TAG, "STEERING VALUE IS %d", steering_input);
    config_.steering_servo.setPosition(steering_input);
}

void Controller::control_motors(rclink::channel_value_t throttle_input, bool reverse) {
    const auto channels = rclink::Receiver::instance().get_channels();
    const bool m1 = channels.ch13.scaled > 1975;
    const bool m2 = channels.ch14.scaled > 1975;
    const bool m3 = channels.ch15.scaled > 1975;
    const bool m4 = channels.ch16.scaled > 1975;

    // ESP_LOGI(TAG, "\n%d %d\n%d %d – %d, %d", m2, m1, m3, m4, throttle_input, reverse);

    int16_t dshot_value;
    if (reverse) {
        dshot_value = 1048 + throttle_input / 2;
    } else {
        dshot_value = 48 + throttle_input / 2;
    }


    dshot_value = std::clamp(dshot_value, static_cast<int16_t>(DSHOT_THROTTLE_MIN),
                                          static_cast<int16_t>(DSHOT_THROTTLE_MAX));

    // ESP_LOGI(TAG, "%d", dshot_value);

    m1 ? config_.motor_fr.sendThrottle(dshot_value) : config_.motor_fr.sendThrottle(1048);
    m2 ? config_.motor_fl.sendThrottle(dshot_value) : config_.motor_fl.sendThrottle(1048);
    m3 ? config_.motor_rl.sendThrottle(dshot_value) : config_.motor_rl.sendThrottle(1048);
    m4 ? config_.motor_rr.sendThrottle(dshot_value) : config_.motor_rr.sendThrottle(1048);

    motor_states_[0].current_erpm = config_.motor_fl.getErpm() != 65535 ? config_.motor_fl.getErpm() : 0;
    motor_states_[1].current_erpm = config_.motor_fr.getErpm() != 65535 ? config_.motor_fr.getErpm() : 0;
    motor_states_[2].current_erpm = config_.motor_rl.getErpm() != 65535 ? config_.motor_rl.getErpm() : 0;
    motor_states_[3].current_erpm = config_.motor_rr.getErpm() != 65535 ? config_.motor_rr.getErpm() : 0;

    // Send RPM telemetry every 10 cycles (100ms at 100Hz)
    if (control_cycle_count_ % 10 == 0 && rclink::Receiver::instance().supports_telemetry()) {
        float erpm_to_rpm = DShotRMT::getErpmToRpmRatio(config_.motor_poles);
        rclink::RpmTelemetry rpm = {
            .source_id = 0,
            .rpm_values = {
                static_cast<int32_t>(motor_states_[0].current_erpm * erpm_to_rpm),
                static_cast<int32_t>(motor_states_[1].current_erpm * erpm_to_rpm),
                static_cast<int32_t>(motor_states_[2].current_erpm * erpm_to_rpm),
                static_cast<int32_t>(motor_states_[3].current_erpm * erpm_to_rpm),
            },
            .count = 4
        };
        rclink::Receiver::instance().send_rpm(rpm);
    }
}

void Controller::handle_rc_loss() {
    armed_ = false;
    config_.steering_servo.setFailsafe();
    // emergency_stop();
}

void Controller::emergency_stop() {
    config_.motor_fl.sendCmd(0);
    config_.motor_fr.sendCmd(0);
    config_.motor_rl.sendCmd(0);
    config_.motor_rr.sendCmd(0);

    for (auto& state : motor_states_) {
        state.current_erpm = 0;
    }
}

} // namespace vehicle
