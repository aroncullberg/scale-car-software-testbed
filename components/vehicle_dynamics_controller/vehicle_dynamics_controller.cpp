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

    while(true) {
        if (sbus_instance.getSbus().quality.valid_signal == false) {
            if (xTaskGetTickCount() % 100 == 0) {
                ESP_LOGE(TAG, "Invlaid sbus signal");
            }
        } else {
            controller->updateSteering(sbus_instance.getSbus().channels[ch_steering]);        
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
