#include "vehicle_dynamics_controller.h"
#include "esp_log.h"
#include "esp_check.h"

static const char* TAG = "VehicleDynamics";

VehicleDynamicsController::VehicleDynamicsController(const Config& config)
    : config_(config),
      steering_servo_(config.steering_servo),
      esc_driver_() {}

VehicleDynamicsController::~VehicleDynamicsController() {
    stop();
}

esp_err_t VehicleDynamicsController::init() {
    // Initialize steering servo
    esp_err_t err = steering_servo_.setPosition(0.0f); // Center the servo
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to initialize steering servo");

    // Initialize ESC driver
    err = esc_driver_.initialize(config_.esc_config);
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to initialize ESC driver");

    return ESP_OK;
}

esp_err_t VehicleDynamicsController::start() {
    if (is_running_) {
        return ESP_OK;
    }

    // Start ESC driver
    esp_err_t err = esc_driver_.start();
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to start ESC driver");

    // Run motor initialization sequence
    
    // Create the controller task
    BaseType_t ret = xTaskCreate(
        controllerTask,
        "veh_dynamics",
        config_.task_stack_size,
        this,
        config_.task_priority,
        &task_handle_
    );
    

    ESP_RETURN_ON_ERROR(esc_driver_.arm_motor(0, true),TAG, "Failed to arm motor %zu", 0);
    ESP_RETURN_ON_ERROR(esc_driver_.arm_motor(1, true),TAG, "Failed to arm motor %zu", 1);
    ESP_RETURN_ON_ERROR(esc_driver_.arm_motor(2, true),TAG, "Failed to arm motor %zu", 2);
    ESP_RETURN_ON_ERROR(esc_driver_.arm_motor(3, true),TAG, "Failed to arm motor %zu", 3);

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

    // Stop all motors
    const size_t num_motors = config_.esc_config.gpio_nums.size();
    for(size_t i = 0; i < num_motors; ++i) {
        esc_driver_.set_throttle(i, 0);
        esc_driver_.arm_motor(i, false);
    }

    // Clean up task
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    is_running_ = false;
    return ESP_OK;
}

esp_err_t VehicleDynamicsController::initializeMotors() {
    ESP_LOGI(TAG, "Starting motor initialization sequence");
    
    const size_t num_motors = config_.esc_config.gpio_nums.size();
    const uint16_t test_throttle = 100;  // Low throttle for testing
    const TickType_t motor_delay = pdMS_TO_TICKS(2500);

    // Arm all motors
    // for(size_t i = 0; i < num_motors; ++i) {
    //     ESP_RETURN_ON_ERROR(
    //         esc_driver_.arm_motor(i, true),
    //         TAG, "Failed to arm motor %zu", i
    //     );
    //     // vTaskDelay(pdMS_TO_TICKS(1000));
    // }
    ESP_RETURN_ON_ERROR(esc_driver_.arm_motor(0, true),TAG, "Failed to arm motor %zu", 0);
    ESP_RETURN_ON_ERROR(esc_driver_.arm_motor(1, true),TAG, "Failed to arm motor %zu", 1);
    ESP_RETURN_ON_ERROR(esc_driver_.arm_motor(2, true),TAG, "Failed to arm motor %zu", 2);
    ESP_RETURN_ON_ERROR(esc_driver_.arm_motor(3, true),TAG, "Failed to arm motor %zu", 3);

    // Test motors sequentially
    for(size_t i = 0; i < num_motors; ++i) {
        ESP_LOGI(TAG, "Testing motor %zu", i);
        
        // Set test throttle
        ESP_RETURN_ON_ERROR(
            esc_driver_.set_throttle(i, test_throttle),
            TAG, "Failed to set motor %zu throttle", i
        );
        vTaskDelay(motor_delay);

        // Reset to zero
        ESP_RETURN_ON_ERROR(
            esc_driver_.set_throttle(i, 0),
            TAG, "Failed to zero motor %zu", i
        );
        vTaskDelay(motor_delay);
    }

    ESP_LOGI(TAG, "Motor initialization completed");
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

    float throttle_position = sbus_data.channels[static_cast<size_t>(sensor::SbusChannel::THROTTLE)];
    uint16_t throttle = static_cast<uint16_t>(throttle_position * 1000); 

    ESP_RETURN_ON_ERROR(
        esc_driver_.set_throttle(0, throttle),
        TAG, "Failed to set motor %zu throttle", 0
    );
    ESP_RETURN_ON_ERROR(
        esc_driver_.set_throttle(1, throttle),
        TAG, "Failed to set motor %zu throttle", 1
    );
    ESP_RETURN_ON_ERROR(
        esc_driver_.set_throttle(2, throttle),
        TAG, "Failed to set motor %zu throttle", 2
    );
    ESP_RETURN_ON_ERROR(
        esc_driver_.set_throttle(3, throttle),
        TAG, "Failed to set motor %zu throttle", 3
    );
    return ESP_OK;
}

esp_err_t VehicleDynamicsController::updateSteering() {
    // Get latest SBUS data
    sensor::SbusData sbus_data = VehicleData::instance().getSbus();
    
    // Process steering channel
    float steering_position = sbus_data.channels[static_cast<size_t>(sensor::SbusChannel::STEERING)];
    
    // Set servo position
    esp_err_t err = steering_servo_.setPosition(steering_position);
    ESP_RETURN_ON_ERROR(err, TAG, "Steering update failed");

    return ESP_OK;
}
