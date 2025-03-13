#include "vdc.h"

#include <sys/stat.h>
#include <cinttypes>

#include "sensor_types.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_task_wdt.h"
#include "config_manager.h"

VehicleDynamicsController::VehicleDynamicsController(const Config& config)
    : config_(config),
      steering_servo_(config.steering_servo) {

    callback_ = [this] { this->updateFromConfig(); };
    ConfigManager::instance().registerCallback(callback_);
}

VehicleDynamicsController::~VehicleDynamicsController() {
    stop();
}

esp_err_t VehicleDynamicsController::init() {
    esp_err_t err = steering_servo_.setPosition(sensor::Servo::NEUTRAL_POSITION);
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to initialize steering servo");

    err = esc_driver_.init(config_.esc_config);
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to initialize ESC driver");

    return ESP_OK;
}

esp_err_t VehicleDynamicsController::start() {
    if (is_running_) {
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(
        controllerTask,
        "veh_dynamics",
        config_.task_stack_size,
        this,
        config_.task_priority,
        &task_handle_,
        1
    );

    ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG, "Failed to create task");

    is_running_ = true;
    return ESP_OK;
}

esp_err_t VehicleDynamicsController::stop() {
    if (!is_running_) {
        return ESP_ERR_INVALID_STATE;
    }

    esc_driver_.set_all_throttles(0);

    // Clean up task
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    is_running_ = false;
    return ESP_OK;
}

void VehicleDynamicsController::updateFromConfig() {
    ESP_LOGI(TAG, "Updating VDC configuration from ConfigManager");

    test_value_ = ConfigManager::instance().getInt("vdc/cmd", test_value_);
    test_delay_ = ConfigManager::instance().getInt("vdc/delay", test_delay_);
    test_repeat_ = ConfigManager::instance().getInt("vdc/repeat", test_repeat_);

    // TODO: make this update stuff for pid ctrlr too
}

namespace {
    float mapChannelToRange(uint16_t channel_value, float min_val, float max_val) {
        if (channel_value > 2000) channel_value = 2000;

        // Map to 0.0-1.0 range
        float normalized = (static_cast<float>(channel_value)) / 2000.0f;

        // Map to target range
        return min_val + normalized * (max_val - min_val);
    }
}

void VehicleDynamicsController::controllerTask(void* arg) {
    auto* controller = static_cast<VehicleDynamicsController*>(arg);
    TickType_t last_wake_time = xTaskGetTickCount();

    const VehicleData &vehicle_data = VehicleData::instance();

    constexpr auto ch_throttle = static_cast<size_t>(sensor::SbusChannel::THROTTLE);
    constexpr auto ch_steering = static_cast<size_t>(sensor::SbusChannel::STEERING);
    constexpr auto ch_turnrate = static_cast<size_t>(sensor::SbusChannel::AUX1);
    constexpr auto ch_gyro_strength = static_cast<size_t>(sensor::SbusChannel::AUX2);
    constexpr auto toggle_pidloop = static_cast<size_t>(sensor::SbusChannel::AUX7);
    constexpr auto arm_switch = static_cast<size_t>(sensor::SbusChannel::AUX8);
    constexpr auto ch_debug = static_cast<size_t>(sensor::SbusChannel::AUX9);
    constexpr auto ch_arm = static_cast<size_t>(sensor::SbusChannel::AUX10);

    uint16_t prev_ch_values[16] = {};
    std::ranges::fill(prev_ch_values, 1000);
    prev_ch_values[0] = 0;

    controller->referenceOrientation_ = getCurrentOrientation(vehicle_data.getImu());

    while(true) {
        const float deltaTime = static_cast<float>(controller->config_.task_period) / 1000.0f;

        const sensor::SbusData& sbus_data = vehicle_data.getSbus();
        const sensor::ImuData& imu_data = vehicle_data.getImu();

        static const uint8_t count = 0;

        if (!sbus_data.quality.valid_signal) {
            ESP_LOGW(TAG, "Invalid SBUS signal");
            controller->steering_servo_.setPosition(sensor::Servo::FAILSAFE_POSITION);
            controller->esc_driver_.set_all_throttles(sensor::Motor::FAILSAFE_THROTTLE); // TODO: Change this to be a realfailsafe where the rmt driver shuts off so esc just shuts down (i hope they do atelast)
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        bool pid_enabled = sbus_data.channels_scaled[toggle_pidloop] > 1900;
        if (pid_enabled && controller->pid_state_ == PidState::DISABLED) {
            controller->pid_state_ = PidState::ACTIVE;
            ESP_LOGI(TAG, "PID loop enabled (switch)");
        } else if (!pid_enabled && (controller->pid_state_ == PidState::ACTIVE ||
                   controller->pid_state_ == PidState::SUSPENDED)) {
            controller->pid_state_ = PidState::DISABLED;
            ESP_LOGI(TAG, "PID loop disabled (switch)");
        }

        uint16_t turnrate_ch = sbus_data.channels_scaled[ch_turnrate];
        float new_turnrate = mapChannelToRange(turnrate_ch, 1.0f, 5.0f);
        if (fabsf(new_turnrate - controller->pid_turnrate_) > 0.2f) {
            ESP_LOGI(TAG, "Turn rate changed: %.2f -> %.2f (ch: %u)",
                    controller->pid_turnrate_,
                    new_turnrate, turnrate_ch);
            controller->pid_turnrate_ = new_turnrate;
        }

        // Gyro strength (0.0-1.0 range)
        uint16_t gyro_ch = sbus_data.channels_scaled[ch_gyro_strength];
        float new_gyro_strength = mapChannelToRange(gyro_ch, 0.0f, 1.0f);
        if (fabsf(new_gyro_strength - controller->gyroConfig_.strength) > 0.02f) {
            ESP_LOGI(TAG, "Gyro strength changed: %.2f -> %.2f (ch: %u)",
                    controller->gyroConfig_.strength,
                    new_gyro_strength, gyro_ch);
            controller->gyroConfig_.strength = new_gyro_strength;
        }

        const bool throttleActive = sbus_data.channels_scaled[0] > 10;
        const uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (throttleActive) {
            controller->lastThrottleActiveTime_ = currentTime;
            if (controller->pid_state_ == PidState::SUSPENDED) {
                controller->pid_state_ = PidState::ACTIVE;
                ESP_LOGI(TAG, "PID reactivated due to throttle application");
            }
        } else {
            // If throttle has been inactive for the timeout period, reset reference
            if ((currentTime - controller->lastThrottleActiveTime_) > controller->gyroConfig_.resetTimeoutMs && controller->pid_state_ == PidState::ACTIVE) {
                controller->pid_state_ = PidState::SUSPENDED;
                controller->referenceOrientation_ = getCurrentOrientation(imu_data);
                controller->targetHeading_ = 0.0f;
                controller->headingPid_.integral = 0.0f;  // Reset integral term
                ESP_LOGI(TAG, "PID suspended due to throttle cut for %ldms (%ld)", controller->gyroConfig_.resetTimeoutMs, (currentTime - controller->lastThrottleActiveTime_));
            }
        }

        controller->updateGyroAssistance(deltaTime, sbus_data, imu_data);

        // controller->updateSteering(sbus_data.channels_scaled[ch_steering]);

        if (sbus_data.channels_scaled[ch_debug] > 1900) {
            ESP_LOGI(TAG, "Debug command received");
            controller->esc_driver_.debug(
                controller->test_value_,
                controller->test_delay_,
                controller->test_repeat_
            );
            vTaskDelay(pdMS_TO_TICKS(350));
        }

        if (sbus_data.channels_scaled[ch_arm] > 1900) {
            ESP_LOGI(TAG, "Arm command received");
            controller->esc_driver_.arm_all();
            vTaskDelay(pdMS_TO_TICKS(350));
        }


        if (sbus_data.channels_scaled[arm_switch] > 1900 && !controller->armed_) {
            controller->armed_ = true;
            ESP_LOGI(TAG, "Armed!");
        } else if (sbus_data.channels_scaled[arm_switch] < 1900 && controller->armed_) {
            controller->armed_ = false;
            ESP_LOGI(TAG, "Disarmed (or is it?)");
            controller->esc_driver_.set_all_throttles(sensor::Motor::FAILSAFE_THROTTLE);
        }

        if (controller->armed_) {
            controller->updateThrottle(sbus_data.channels_scaled[ch_throttle]);
        }

        vTaskDelayUntil(&last_wake_time, controller->config_.task_period);
    }
}

esp_err_t VehicleDynamicsController::updateThrottle(sensor::channel_t throttle_value) {
    // ESP_LOGI(TAG, "Throttle value: %d", throttle_value);
    // return ESP_OK;
    return esc_driver_.set_all_throttles(throttle_value);
}

esp_err_t VehicleDynamicsController::updateSteering(uint16_t steering_value) {
    return steering_servo_.setPosition(steering_value);
}


Quaternion VehicleDynamicsController::getCurrentOrientation(const sensor::ImuData& imu_data) {
    return Quaternion::fromGameVector(imu_data.quat6_x, imu_data.quat6_y, imu_data.quat6_z);
}

float VehicleDynamicsController::calculateHeadingError(const sensor::ImuData& imu_data) const {
    Quaternion currentOrientation = getCurrentOrientation(imu_data);

    const float baseHeading = Quaternion::headingDifference(referenceOrientation_, currentOrientation);

    float error = targetHeading_ - baseHeading;
    while (error > M_PI) error -= 2.0f * M_PI;
    while (error < -M_PI) error += 2.0f * M_PI;

    return -error;
}

void VehicleDynamicsController::logOrientationData(const char* prefix, const Quaternion& orientation) {
    // Log quaternion components
    // ESP_LOGI(TAG, "%s Quaternion: w=%.4f, x=%.4f, y=%.4f, z=%.4f",
    //          prefix, orientation.w, orientation.x, orientation.y, orientation.z);

    // Calculate and log heading in degrees
    float heading = atan2f(2.0f * (orientation.w * orientation.z + orientation.x * orientation.y),
                          1.0f - 2.0f * (orientation.y * orientation.y + orientation.z * orientation.z));

    ESP_LOGI(TAG, "%s Heading: %.2f degrees", prefix, heading * 180.0f / M_PI);
}


float VehicleDynamicsController::computePID(float error, float deltaTime) {
    // P term
    float output = error * headingPid_.kP;

    // I term (with anti-windup)
    headingPid_.integral += error * deltaTime;

    // Basic anti-windup (clamp integral)
    constexpr float MAX_INTEGRAL = 1.0f;
    if (headingPid_.integral > MAX_INTEGRAL) headingPid_.integral = MAX_INTEGRAL;
    if (headingPid_.integral < -MAX_INTEGRAL) headingPid_.integral = -MAX_INTEGRAL;

    output += headingPid_.integral * headingPid_.kI;

    // D term
    float derivative = (error - headingPid_.previousError) / deltaTime;
    output += derivative * headingPid_.kD;

    // Store error for next iteration
    headingPid_.previousError = error;

    return output;
}


// TODO: Convert to retrun esp_err_t
void VehicleDynamicsController::updateGyroAssistance(float deltaTime, const sensor::SbusData& sbus_data, const sensor::ImuData& imu_data) {
    if (pid_state_ == PidState::DISABLED || pid_state_ == PidState::SUSPENDED) {
        steering_servo_.setPosition(sbus_data.channels_scaled[1]); // TODO: change to return
        return;
    }

    // Normalize steering input to -1.0 to 1.0
    // Assuming steering is on channel 1, adjust as needed
    float steeringInput = static_cast<float>(sbus_data.channels_scaled[1] - 1000) / 1000.0f;

    static const uint8_t count = 0;
    // if (count % 100 == 0) {
    //     ESP_LOGI(TAG, "steeringInput: %7.4f", steeringInput);
    // }

    // ESP_LOGI(TAG, "Steering input: %7.4f", steeringInput);

    // Update target heading based on steering input
    if (fabs(steeringInput) > 0.05f) {  // Small deadzone
        // Change rate proportional to stick deflection
        float headingChangeRate = steeringInput * pid_turnrate_;  // Adjust multiplier for sensitivity
        targetHeading_ += headingChangeRate * deltaTime;

        // Limit target heading to ±90 degrees (π/2 radians)
        if (targetHeading_ > M_PI_2) targetHeading_ = M_PI_2;
        if (targetHeading_ < -M_PI_2) targetHeading_ = -M_PI_2;

        // Normalize target heading
        while (targetHeading_ > M_PI) targetHeading_ -= 2.0f * M_PI;
        while (targetHeading_ < -M_PI) targetHeading_ += 2.0f * M_PI;
    }

    // Calculate heading error
    float headingError = calculateHeadingError(imu_data);

    // Compute PID correction
    float correction = computePID(headingError, deltaTime);

    // Apply correction to steering (mixed with direct input)
    float directFactor = 1.0f - gyroConfig_.strength;
    float gyroFactor = gyroConfig_.strength;

    float finalSteering = steeringInput * directFactor - correction * gyroFactor;

    // Clamp to valid range
    if (finalSteering > 2.0f) finalSteering = 2.0f;
    if (finalSteering < -2.0f) finalSteering = -2.0f;

    static int debug_counter = 0;
    if (++debug_counter % 50 == 0) {  // Adjust rate as needed
        Quaternion currentOrientation = getCurrentOrientation(imu_data);

        logOrientationData("CURRENT", currentOrientation);
        logOrientationData("REFERENCE", referenceOrientation_);

        // Calculate and display heading difference directly
        float headingDiff = Quaternion::headingDifference(referenceOrientation_, currentOrientation) * 180.0f / M_PI;
        float targetHeadingDeg = targetHeading_ * 180.0f / M_PI;

        ESP_LOGI(TAG, "HEADING: Target=%.2f°, Current diff=%.2f°, Error=%.2f°",
                 targetHeadingDeg, headingDiff, headingError * 180.0f / M_PI);

        ESP_LOGI(TAG, "STEERING: Input=%.2f, Correction=%.2f, Final=%.2f",
                 steeringInput, correction, finalSteering);
        ESP_LOGI(TAG, "");
    }



    // Convert to servo range (1000-2000) and set steering
    const auto servoValue = static_cast<uint16_t>(1000 + finalSteering * 500);
    esp_err_t err = steering_servo_.setPosition(servoValue);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set steering servo position: %d", err);
    }
}
