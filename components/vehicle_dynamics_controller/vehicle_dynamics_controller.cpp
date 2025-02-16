#include "vehicle_dynamics_controller.h"
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

    ESP_LOGI(TAG, "Vehicle dynamics controller initialized");
    return ESP_OK;
}

esp_err_t VehicleDynamicsController::start() {
    if (is_running_) {
        return ESP_OK;
    }

    esp_err_t err = esc_driver_.start();
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to start ESC driver");

    // Create the controller task
    BaseType_t ret = xTaskCreate(
        controllerTask,
        "veh_dynamics",
        config_.task_stack_size,
        this,
        config_.task_priority,
        &task_handle_
    );
    

    ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG, "Failed to create task");
    
    // err = initializeMotors();
    // ESP_RETURN_ON_ERROR(err, TAG, "Motor initialization failed");

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

    while(true) {
        controller->updateSteering();
        controller->updateThrottle();
        vTaskDelayUntil(&last_wake_time, controller->config_.task_period);
    }
}

esp_err_t VehicleDynamicsController::updateThrottle() {
    sensor::SbusData sbus_data = VehicleData::instance().getSbus();

    uint16_t throttle = sbus_data.channels[static_cast<size_t>(sensor::SbusChannel::THROTTLE)];

    return esc_driver_.set_all_throttles(throttle);

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

esp_err_t VehicleDynamicsController::updateSteering() {
    // Get latest SBUS data
    sensor::SbusData sbus_data = VehicleData::instance().getSbus();
    
    // Process steering channel
    uint16_t steering_position = sbus_data.channels[static_cast<size_t>(sensor::SbusChannel::STEERING)];
    
    // Set servo position
    esp_err_t err = steering_servo_.setPosition(steering_position);
    ESP_RETURN_ON_ERROR(err, TAG, "Steering update failed");

    return ESP_OK;
}
