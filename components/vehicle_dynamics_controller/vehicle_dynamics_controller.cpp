#include "vehicle_dynamics_controller.h"
#include "esp_log.h"

VehicleDynamicsController::VehicleDynamicsController(const Config& config)
    : config_(config)
    , steering_servo_(config.steering_servo) {
}

VehicleDynamicsController::~VehicleDynamicsController() {
    stop();
}

esp_err_t VehicleDynamicsController::init() {
    esp_err_t err = steering_servo_.setPosition(0.0f); // Center the servo
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize steering servo: %s", esp_err_to_name(err));
        return err;
    }
    
    return ESP_OK;
}

esp_err_t VehicleDynamicsController::start() {
    if (is_running_) {
        return ESP_OK;
    }

    // Create the controller task
    BaseType_t ret = xTaskCreate(
        controllerTask,
        "veh_dynamics",
        config_.task_stack_size,
        this,
        config_.task_priority,
        &task_handle_
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create vehicle dynamics task");
        return ESP_FAIL;
    }

    is_running_ = true;
    return ESP_OK;
}

esp_err_t VehicleDynamicsController::stop() {
    if (!is_running_) {
        return ESP_OK;
    }

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

    while (true) {
        controller->updateSteering();
        vTaskDelayUntil(&last_wake_time, controller->config_.task_period);
    }
}

esp_err_t VehicleDynamicsController::updateSteering() {
    // Get latest SBUS data from the data pool
    sensor::SbusData sbus_data = VehicleData::instance().getSbus();
    
    // SBUS steering channel is channel 1 (array index 1)
    // Value is already scaled from -1.0 to 1.0 by the SBUS driver
    float steering_position = sbus_data.channels[static_cast<size_t>(sensor::SbusChannel::STEERING)];
    
    // Set the servo position directly
    esp_err_t err = steering_servo_.setPosition(steering_position);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set steering position: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}