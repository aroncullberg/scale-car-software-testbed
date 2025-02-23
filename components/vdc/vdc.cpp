#include "vdc.h"
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
    esp_err_t err;
    
    BaseType_t ret = xTaskCreate(
        controllerTask,
        "veh_dynamics",
        config_.task_stack_size,
        this,
        config_.task_priority,
        &task_handle_
    );
    

    ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG, "Failed to create task");
     
    is_running_ = true;
    return ESP_OK;
}

esp_err_t VehicleDynamicsController::stop() {
    if (!is_running_) {
        return ESP_OK;
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


void VehicleDynamicsController::controllerTask(void* arg) {
    auto* controller = static_cast<VehicleDynamicsController*>(arg);
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t log_interval = 1000;
    VehicleData &sbus_instance = VehicleData::instance();

    size_t ch_throttle = static_cast<size_t>(sensor::SbusChannel::THROTTLE);
    size_t ch_steering = static_cast<size_t>(sensor::SbusChannel::STEERING);

    controller->referenceOrientation_ = controller->getCurrentOrientation();

    while(true) {
        float deltaTime = controller->config_.task_period / 1000.0f;

        if (sbus_instance.getSbus().quality.valid_signal == false) {
            continue;
            // if (xTaskGetTickCount() % 100 == 0) {
            //     ESP_LOGW(TAG, "Invlaid sbus signal");
            // }
        }
        
        controller->updateGyroAssistance(deltaTime);
    
        if (sbus_instance.getSbus().channels[ch_throttle] > 1010 && !controller->esc_driver_.is_armed()) {
            esp_err_t err = esp_task_wdt_reset();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to reset task watchdog! Error: %d", err);
            }
            continue;
        } else if (!controller->esc_driver_.is_armed()){
            controller->esc_driver_.arm_all();
        }
        if (!controller->esc_driver_.is_armed()) {
            if (sbus_instance.getSbus().channels[ch_throttle] > 1010) {
                continue;
            } else {
                controller->esc_driver_.arm_all();
            }
        }
        controller->updateThrottle(sbus_instance.getSbus().channels[ch_throttle]);        
        
        vTaskDelayUntil(&last_wake_time, controller->config_.task_period);
    }
}

esp_err_t VehicleDynamicsController::updateThrottle(uint16_t throttle_value) {
    // sensor::SbusData sbus_data = VehicleData::instance().getSbus();

    // uint16_t throttle = sbus_data.channels[static_cast<size_t>(sensor::SbusChannel::THROTTLE)];
    // if (999 >= throttle || throttle >= 2001) {
    //     ESPLOG
    // }
    // ESP_RETURN_ON_FALSE(999 <= throttle_value && throttle_value <= 1300, ESP_ERR_INVALID_STATE, TAG, "Throttle out of bounds");

    return esc_driver_.set_all_throttles(throttle_value);

    // ESP_RETURN_ON_ERROR(
    //     esc_driver_.set_throttle(EscDriver::MotorPosition::FRONT_RIGHT, throttle),
    //     TAG, "Failed to set front right motor throttle"
    // );
    // ESP_RETURN_ON_ERROR(
    //     esc_driver_.set_throttle(EscDriver::MotorPosition::FRONT_LEFT, throttle),
    //     TAG, "Failed to set front left motor throttle"
    // );
    // ESP_RETURN_ON_ERROR(
    //     esc_driver_.set_throttle(EscDriver::MotorPosition::REAR_LEFT, throttle),
    //     TAG, "Failed to set rear left motor throttle"
    // );
    // ESP_RETURN_ON_ERROR(
    //     esc_driver_.set_throttle(EscDriver::MotorPosition::REAR_RIGHT, throttle),
    //     TAG, "Failed to set rear right motor throttle"
    // );

    // return ESP_OK;
}

Quaternion VehicleDynamicsController::getCurrentOrientation() {
    const auto& imuData = VehicleData::instance().getImu();
    return Quaternion::fromGameVector(imuData.quat6_x, imuData.quat6_y, imuData.quat6_z);
}

float VehicleDynamicsController::calculateHeadingError() {
    Quaternion currentOrientation = getCurrentOrientation();
    
    float baseHeading = Quaternion::headingDifference(referenceOrientation_, currentOrientation);
    
    // wrap arund causes issue
    float error = targetHeading_ - baseHeading;
    
    // Normalize to -PI to PI
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

void VehicleDynamicsController::updateReferenceOrientation() {
    const auto& sbusData = VehicleData::instance().getSbus();
    
    // Check if throttle is active (assuming throttle is on channel 0)
    // Adjust channel index and threshold based on your actual setup
    bool throttleActive = (sbusData.channels[0] > 1010);
    
    uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    if (throttleActive) {
        lastThrottleActiveTime_ = currentTime;
    } else {
        // If throttle has been inactive for the timeout period, reset reference
        if ((currentTime - lastThrottleActiveTime_) > gyroConfig_.resetTimeoutMs) {
            referenceOrientation_ = getCurrentOrientation();
            targetHeading_ = 0.0f;
            headingPid_.integral = 0.0f;  // Reset integral term
        }
    }
}

void VehicleDynamicsController::updateGyroAssistance(float deltaTime) {
    if (!gyroConfig_.enabled) {
        return;
    }
    
    // Update reference if needed
    updateReferenceOrientation();
    
    // Get current steering input from SBUS
    const auto& sbusData = VehicleData::instance().getSbus();
    
    // Normalize steering input to -1.0 to 1.0
    // Assuming steering is on channel 1, adjust as needed
    float steeringInput = (sbusData.channels[1] - 1500) / 500.0f;
    
    // Update target heading based on steering input
    if (fabs(steeringInput) > 0.05f) {  // Small deadzone
        // Change rate proportional to stick deflection
        float headingChangeRate = -steeringInput * 1.5f;  // Adjust multiplier for sensitivity
        targetHeading_ += headingChangeRate * deltaTime;
        
        // Normalize target heading
        while (targetHeading_ > M_PI) targetHeading_ -= 2.0f * M_PI;
        while (targetHeading_ < -M_PI) targetHeading_ += 2.0f * M_PI;
    }
    
    // Calculate heading error
    float headingError = calculateHeadingError();
    
    // Compute PID correction
    float correction = computePID(headingError, deltaTime);
    
    // Apply correction to steering (mixed with direct input)
    float directFactor = 1.0f - gyroConfig_.strength;
    float gyroFactor = gyroConfig_.strength;
    
    
    // Final steering is a mix of direct control and gyro correction
    
    float finalSteering = steeringInput * directFactor - correction * gyroFactor;
    bool throttleActive = (sbusData.channels[0] > 1010);

    // if (!throttleActive) {
    //     finalSteering = steeringInput;
    // }

     
    TickType_t tick = xTaskGetTickCount();
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
    uint16_t servoValue = 1500 + finalSteering * 500;
    steering_servo_.setPosition(servoValue);
}
