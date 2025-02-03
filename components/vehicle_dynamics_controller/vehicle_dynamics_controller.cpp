#include "vehicle_dynamics_controller.h"
#include "esp_log.h"

namespace control {

VehicleDynamicsController::VehicleDynamicsController(const Config& config)
    : config_(config) {
}

VehicleDynamicsController::~VehicleDynamicsController() {
    stop();
}

esp_err_t VehicleDynamicsController::init() {
    ESP_LOGI(TAG, "Initializing Vehicle Dynamics Controller");
    return ESP_OK;
}

esp_err_t VehicleDynamicsController::start() {
    if (is_running_) {
        ESP_LOGW(TAG, "Vehicle Dynamics Controller already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(
        controllerTask,
        "vdc_task",
        4096,                // Stack size
        this,               // Parameter
        config_.task_priority,
        &task_handle_,
        config_.task_core_id
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Vehicle Dynamics Controller task");
        return ESP_FAIL;
    }

    is_running_ = true;
    ESP_LOGI(TAG, "Vehicle Dynamics Controller started");
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
    ESP_LOGI(TAG, "Vehicle Dynamics Controller stopped");
    return ESP_OK;
}

VehicleDynamicsController::MotorThrottles 
VehicleDynamicsController::calculateMotorThrottles(float base_throttle, float front_rear_split) const {
    MotorThrottles throttles;
    
    // Calculate front and rear power based on split
    float front_power = base_throttle * (front_rear_split);
    float rear_power = base_throttle * (1.0f - front_rear_split);

    // Apply to all wheels
    throttles.front_left = front_power;
    throttles.front_right = front_power;
    throttles.rear_left = rear_power;
    throttles.rear_right = rear_power;

    return throttles;
}

void VehicleDynamicsController::controllerTask(void* parameters) {
    auto* controller = static_cast<VehicleDynamicsController*>(parameters);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        // Get throttle input from SBUS
        sensor::SbusData sbus = VehicleData::instance().getSbus();
        
        // Read base throttle (assuming channel 0 is throttle)
        float base_throttle = sbus.channels[0];
        
        // Read front/rear split from configured AUX channel
        float front_rear_split = controller->config_.default_front_rear_split;
        if (controller->config_.front_rear_split_channel < 16) {
            front_rear_split = (sbus.channels[controller->config_.front_rear_split_channel] + 1.0f) * 0.5f; // Convert -1 to 1 into 0 to 1
        }

        // Calculate individual motor throttles
        controller->current_throttles_ = controller->calculateMotorThrottles(base_throttle, front_rear_split);

        // Update motor throttles in data pool
        // TODO: Add motor throttles to data pool structure and update here

        // Wait for next cycle
        vTaskDelayUntil(&last_wake_time, controller->config_.task_period);
    }
}

} // namespace control