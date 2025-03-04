#include "vdc.h"

#include <sys/stat.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_task_wdt.h"

VehicleDynamicsController::VehicleDynamicsController(const Config& config)
    : config_(config),
      steering_servo_(config.steering_servo),
      esc_driver_() {}

VehicleDynamicsController::~VehicleDynamicsController() {
    stop();
}

esp_err_t VehicleDynamicsController::init() {
    esp_err_t err = steering_servo_.setPosition(1500); // Center the servo
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to initialize steering servo");

    err = esc_driver_.init(config_.esc_config);
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to initialize ESC driver");

    return ESP_OK;
}

esp_err_t VehicleDynamicsController::start() {
    if (is_running_) {
        return ESP_OK;
    }

    const BaseType_t ret = xTaskCreatePinnedToCore(
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

namespace {
    float mapChannelToRange(uint16_t channel_value, float min_val, float max_val) {
        if (channel_value < 1000) channel_value = 1000;
        if (channel_value > 2000) channel_value = 2000;

        // Map to 0.0-1.0 range
        float normalized = (static_cast<float>(channel_value) - 1000) / 1000.0f;

        // Map to target range
        return min_val + normalized * (max_val - min_val);
    }

    // Process a tristate switch input
    // Returns: -1 for decrease, 0 for no change, 1 for increase
    int processTristate(const uint16_t channel_value) {
        if (channel_value < 1300) return -1;      // Low position
        if (channel_value > 1700) return 1;  // High position
        return 0;
    }
}



// TODO: change the loop to not do repeat data fetches
void VehicleDynamicsController::controllerTask(void* arg) {
    auto* controller = static_cast<VehicleDynamicsController*>(arg);
    TickType_t last_wake_time = xTaskGetTickCount();

    // const VehicleData &sbus_instance = VehicleData::instance();
    const VehicleData &vehicle_data = VehicleData::instance();

    constexpr auto ch_throttle = static_cast<size_t>(sensor::SbusChannel::THROTTLE);
    constexpr auto ch_steering = static_cast<size_t>(sensor::SbusChannel::STEERING);
    constexpr auto ch_turnrate = static_cast<size_t>(sensor::SbusChannel::AUX1);
    constexpr auto ch_gyro_strength = static_cast<size_t>(sensor::SbusChannel::AUX2);
    constexpr auto ch_kp = static_cast<size_t>(sensor::SbusChannel::AUX3);
    constexpr auto ch_kd = static_cast<size_t>(sensor::SbusChannel::AUX4);
    constexpr auto ch_ki = static_cast<size_t>(sensor::SbusChannel::AUX5);
    constexpr auto ch_reset_timeout = static_cast<size_t>(sensor::SbusChannel::AUX6);
    constexpr auto toggle_pidloop = static_cast<size_t>(sensor::SbusChannel::AUX7);
    constexpr auto arm_switch = static_cast<size_t>(sensor::SbusChannel::AUX8);
    constexpr auto ch_arm1 = static_cast<size_t>(sensor::SbusChannel::AUX9);
    constexpr auto ch_arm2 = static_cast<size_t>(sensor::SbusChannel::AUX10);

    uint16_t prev_ch_values[16] = {};
    std::ranges::fill(prev_ch_values, 1500);

    controller->referenceOrientation_ = VehicleDynamicsController::getCurrentOrientation(vehicle_data.getImu());

    while(true) {
        const float deltaTime = static_cast<float>(controller->config_.task_period) / 1000.0f;
        const sensor::SbusData& sbus_data = vehicle_data.getSbus();
        const sensor::ImuData& imu_data = vehicle_data.getImu();

        if (!sbus_data.quality.valid_signal) {
            if (xTaskGetTickCount() % 99 == 0) {
                ESP_LOGW(TAG, "Invalid sbus signal");
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // Short delay to prevent CPU hogging
            continue;
        }

        // Turn rate coefficient (1-10 range)
        uint16_t turnrate_ch = sbus_data.channels[ch_turnrate];
        float new_turnrate = mapChannelToRange(turnrate_ch, 1.0f, 10.0f);
        if (fabsf(new_turnrate - controller->coefficents_.headingChangeRateCoefficent) > 0.1f) {
            ESP_LOGI(TAG, "Turn rate changed: %.2f -> %.2f (ch: %u)",
                    controller->coefficents_.headingChangeRateCoefficent,
                    new_turnrate, turnrate_ch);
            controller->coefficents_.headingChangeRateCoefficent = new_turnrate;
        }

        // Gyro strength (0.0-1.0 range)
        uint16_t gyro_ch = sbus_data.channels[ch_gyro_strength];
        float new_gyro_strength = mapChannelToRange(gyro_ch, 0.0f, 1.0f);
        if (fabsf(new_gyro_strength - controller->gyroConfig_.strength) > 0.02f) {
            ESP_LOGI(TAG, "Gyro strength changed: %.2f -> %.2f (ch: %u)",
                    controller->gyroConfig_.strength,
                    new_gyro_strength, gyro_ch);
            controller->gyroConfig_.strength = new_gyro_strength;
        }

        // kP adjustment
        int kp_adj = processTristate(sbus_data.channels[ch_kp]);
        if (kp_adj != 0) {
            constexpr float kp_adj_rate = 0.01f;
            float new_kp = controller->headingPid_.kP + static_cast<float>(kp_adj) * kp_adj_rate;
            // Clamp to reasonable range
            if (new_kp <= 0.0f) new_kp = 0.0f;
            if (new_kp >= 10.0f) new_kp = 10.0f;

            if (fabsf(new_kp - controller->headingPid_.kP) > 0.001f) {
                ESP_LOGI(TAG, "kP changed: %.3f -> %.3f (adj: %d)",
                        controller->headingPid_.kP, new_kp, kp_adj);
                controller->headingPid_.kP = new_kp;
            }
        }

        // kI adjustment
        int ki_adj = processTristate(sbus_data.channels[ch_ki]);
        if (ki_adj != 0) {
            constexpr float ki_adj_rate = 0.001f;
            float new_ki = controller->headingPid_.kI + static_cast<float>(ki_adj) * ki_adj_rate;
            // Clamp to reasonable range
            if (new_ki <= 0.0f) new_ki = 0.0f;
            if (new_ki >= 2.0f) new_ki = 2.0f;

            if (fabsf(new_ki - controller->headingPid_.kI) > 0.0001f) {
                ESP_LOGI(TAG, "kI changed: %.4f -> %.4f (adj: %d)",
                        controller->headingPid_.kI, new_ki, ki_adj);
                controller->headingPid_.kI = new_ki;
            }
        }

        // kD adjustment
        int kd_adj = processTristate(sbus_data.channels[ch_kd]);
        if (kd_adj != 0) {
            constexpr float kd_adj_rate = 0.01f;
            float new_kd = controller->headingPid_.kD + kd_adj * kd_adj_rate;
            // Clamp to reasonable range
            if (new_kd <= 0.0f) new_kd = 0.0f;
            if (new_kd >= 2.0f) new_kd = 2.0f;

            if (fabsf(new_kd - controller->headingPid_.kD) > 0.001f) {
                ESP_LOGI(TAG, "kD changed: %.3f -> %.3f (adj: %d)",
                        controller->headingPid_.kD, new_kd, kd_adj);
                controller->headingPid_.kD = new_kd;
            }
        }

        // Reset timeout adjustment
        int timeout_adj = processTristate(sbus_data.channels[ch_reset_timeout]);
        if (timeout_adj != 0) {
            constexpr float timeout_adj_rate = 50.0f;
            uint32_t new_timeout = controller->gyroConfig_.resetTimeoutMs +
                                   timeout_adj * static_cast<int>(timeout_adj_rate);
            // Clamp to reasonable range
            if (new_timeout < 100) new_timeout = 100;
            if (new_timeout > 5000) new_timeout = 5000;

            if (new_timeout != controller->gyroConfig_.resetTimeoutMs) {
                ESP_LOGI(TAG, "Reset timeout changed: %" PRIu32 " -> %" PRIu32 " ms (adj: %d)",
                         controller->gyroConfig_.resetTimeoutMs, new_timeout, timeout_adj);
                controller->gyroConfig_.resetTimeoutMs = new_timeout;
            }
        }



        // NOTE: This is the steering thing, stupid name i know
        // TODO: Change to a more descriptive name

        if (sbus_data.channels[toggle_pidloop] < 1600) {
            controller->updateGyroAssistance(deltaTime, sbus_data, imu_data);
            // ESP_LOGI(TAG, "PID loop enabled");
        } else {
            controller->updateSteering(sbus_data.channels[ch_steering]);
            // ESP_LOGI(TAG, "PID loop disabled");
        }

        const uint16_t throttle_value = sbus_data.channels[ch_throttle];
        // const bool is_armed = controller->esc_driver_.is_armed();
        const bool is_armed = false;

        if (sbus_data.channels[ch_ki] > 1900) {
            controller->esc_driver_.arm1_all();
            vTaskDelay(pdMS_TO_TICKS(350));
        }
        if (sbus_data.channels[ch_ki] > 1900) {
            controller->esc_driver_.arm1_all();
            vTaskDelay(pdMS_TO_TICKS(350));
        }

        if (sbus_data.channels[arm_switch] > 1900) {
            controller->updateThrottle(throttle_value);
        }

        vTaskDelayUntil(&last_wake_time, controller->config_.task_period);
    }
}

esp_err_t VehicleDynamicsController::updateThrottle(uint16_t throttle_value) {
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
    
    return error;
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

void VehicleDynamicsController::updateReferenceOrientation(const sensor::SbusData& sbus_data,
                                                         const sensor::ImuData& imu_data) {
    const bool throttleActive = sbus_data.channels[0] > 1001;
    const uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    if (throttleActive) {
        lastThrottleActiveTime_ = currentTime;
    } else {
        // If throttle has been inactive for the timeout period, reset reference
        if ((currentTime - lastThrottleActiveTime_) > gyroConfig_.resetTimeoutMs) {
            referenceOrientation_ = getCurrentOrientation(imu_data);
            targetHeading_ = 0.0f;
            headingPid_.integral = 0.0f;  // Reset integral term
        }
    }
}

void VehicleDynamicsController::updateGyroAssistance(float deltaTime, const sensor::SbusData& sbus_data, const sensor::ImuData& imu_data) {
    if (!gyroConfig_.enabled) {
        return;
    }

    // Update reference if needed
    updateReferenceOrientation(sbus_data, imu_data);

    // Get current steering input from SBUS

    // Normalize steering input to -1.0 to 1.0
    // Assuming steering is on channel 1, adjust as needed
    float steeringInput = static_cast<float>(sbus_data.channels[1] - 1500) / 500.0f;

    // Update target heading based on steering input
    if (fabs(steeringInput) > 0.05f) {  // Small deadzone
        // Change rate proportional to stick deflection
        float headingChangeRate = -steeringInput * coefficents_.headingChangeRateCoefficent;  // Adjust multiplier for sensitivity
        targetHeading_ += headingChangeRate * deltaTime;
        
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
    
    
    // Final steering is a mix of direct control and gyro correction
    
    float finalSteering = steeringInput * directFactor - correction * gyroFactor;

    // bool throttleActive = (sbusData.channels[0] > 1001);
    // if (!throttleActive) {
    //     finalSteering = steeringInput;
    // }

    // TickType_t tick = xTaskGetTickCount();
    // ESP_LOGI(TAG, "Steering input: %d", static_cast<int>(tick));
    
    // char log_buffer[256];  // Adjust size as needed
    // if (tick % 9 == 0) {
    //     snprintf(log_buffer, sizeof(log_buffer),
    //              "Steering input: %7.4f, directFactor: %7.4f, correction: %7.4f, gyroFactor: %7.4f, finalSteering: %7.4f",
    //              steeringInput, directFactor, correction, gyroFactor, finalSteering);
        
    //     // Print to log
    //     ESP_LOGI(TAG, "%s", log_buffer);

    //     // Send over ESP-NOW, UART, etc.
    // }

    // Clamp to valid range
    if (finalSteering > 1.0f) finalSteering = 1.0f;
    if (finalSteering < -1.0f) finalSteering = -1.0f;
    
    // Convert to servo range (1000-2000) and set steering
    const auto servoValue = static_cast<uint16_t>(1500 + finalSteering * 500);
    esp_err_t err = steering_servo_.setPosition(servoValue);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set steering servo position: %d", err);
    }
}
