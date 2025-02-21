#include "vdc.h"
#include "esp_log.h"
#include "esp_check.h"

VehicleDynamicsController::VehicleDynamicsController(const Config& config)
    : config_(config),
      steering_servo_(config.steering_servo),
      esc_driver_() {}

VehicleDynamicsController::~VehicleDynamicsController() {
    stop();
}

esp_err_t VehicleDynamicsController::init() {
    esp_err_t err = steering_servo_.setPosition(0.0f); // Center the servo
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
    
    err = esc_driver_.start();
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to start ESC driver");

    // esc_driver_.set_all_throttles(1020);

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
    VehicleData &sbus_instance = VehicleData::instance();

    size_t ch_throttle = static_cast<size_t>(sensor::SbusChannel::THROTTLE);
    size_t ch_steering = static_cast<size_t>(sensor::SbusChannel::STEERING);

    controller->referenceOrientation_ = controller->getCurrentOrientation();


    while(true) {
        float deltaTime = controller->config_.task_period / 1000.0f;

        if (sbus_instance.getSbus().quality.valid_signal == false) {
            if (xTaskGetTickCount() % 100 == 0) {
                ESP_LOGE(TAG, "Invlaid sbus signal");
            }
        } else {
            // controller->updateSteering(sbus_instance.getSbus().channels[ch_steering]);  
            controller->updateGyroAssistance(deltaTime);
      
            controller->updateThrottle(sbus_instance.getSbus().channels[ch_throttle]);        
        }
        
        vTaskDelayUntil(&last_wake_time, controller->config_.task_period);

      //   vTaskDela(pdMS_TO_TICKS(1000)));
    }
}

esp_err_t VehicleDynamicsController::updateThrottle(uint16_t throttle_value) {
    // sensor::SbusData sbus_data = VehicleData::instance().getSbus();

    // uint16_t throttle = sbus_data.channels[static_cast<size_t>(sensor::SbusChannel::THROTTLE)];
    // if (999 >= throttle || throttle >= 2001) {
    //     ESPLOG
    // }
    ESP_RETURN_ON_FALSE(999 <= throttle_value || throttle_value <= 1300, ESP_ERR_INVALID_STATE, TAG, "Throttle out of bounds");

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

esp_err_t VehicleDynamicsController::updateSteering(uint16_t steering_position) {
    // Get latest SBUS data
    // sensor::SbusData sbus_data = VehicleData::instance().getSbus();
    
    // Process steering channel
    // uint16_t steering_position = sbus_data.channels[static_cast<size_t>(sensor::SbusChannel::STEERING)];
    
    // Set servo position
    esp_err_t err = steering_servo_.setPosition(steering_position);
    ESP_RETURN_ON_ERROR(err, TAG, "Steering update failed");

    return ESP_OK;
}


Quaternion VehicleDynamicsController::getCurrentOrientation() {
    const auto& imuData = VehicleData::instance().getImu();
    return Quaternion::fromGameVector(imuData.quat6_x, imuData.quat6_y, imuData.quat6_z);
}

float VehicleDynamicsController::calculateHeadingError() {
    Quaternion currentOrientation = getCurrentOrientation();
    
    // Calculate base heading difference from reference
    float baseHeading = Quaternion::headingDifference(referenceOrientation_, currentOrientation);
    
    // Calculate error from target heading (adjust for wrap-around)
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
    bool throttleActive = (sbusData.channels[0] > 1100);
    
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
        float headingChangeRate = steeringInput * 1.5f;  // Adjust multiplier for sensitivity
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
    float finalSteering = steeringInput * directFactor + correction * gyroFactor;
    
    // Clamp to valid range
    if (finalSteering > 1.0f) finalSteering = 1.0f;
    if (finalSteering < -1.0f) finalSteering = -1.0f;
    
    // Convert to servo range (1000-2000) and set steering
    uint16_t servoValue = 1500 + finalSteering * 500;
    steering_servo_.setPosition(servoValue);
}
