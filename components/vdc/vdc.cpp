#include "vdc.h"

#include <sys/stat.h>
#include <cinttypes>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_task_wdt.h"
#include "config_manager.h"

VehicleDynamicsController::VehicleDynamicsController(const Config& config)
    : config_(config),
      steering_servo_(config.steering_servo),
      steering_pid_(config.pid_config) {

    callback_ = [this] { this->updateFromConfig(); };
    ConfigManager::instance().registerCallback(callback_);

    steering_pid_.setConfigCallback([this] { this->updateFromConfig(); }); // TODO: find better way to do this
}

VehicleDynamicsController::~VehicleDynamicsController() {
    stop();
}

esp_err_t VehicleDynamicsController::init() {
    esp_err_t err = steering_servo_.setPosition(1500);
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

void VehicleDynamicsController::enablePID(bool enable) {
    if (enable != use_pidloop_) {
        use_pidloop_ = enable;
        ESP_LOGI(TAG, "PID control %s", enable ? "enabled" : "disabled");

        if (enable) {
            steering_pid_.setState(SteeringPID::ControllerState::ACTIVE);
        } else {
            steering_pid_.setState(SteeringPID::ControllerState::DISABLED);
        }
    }
}

// TODO: Fix this abomination of callback hell
void VehicleDynamicsController::updateFromConfig() {
    ESP_LOGI(TAG, "Updating VDC configuration from ConfigManager");

    SteeringPID::Config pid_config = config_.pid_config;

    pid_config.kP = ConfigManager::instance().getFloat("pid/kp", pid_config.kP);
    pid_config.kI = ConfigManager::instance().getFloat("pid/ki", pid_config.kI);
    pid_config.kD = ConfigManager::instance().getFloat("pid/kd", pid_config.kD);
    pid_config.feedForwardGain = ConfigManager::instance().getFloat("pid/ff", pid_config.feedForwardGain);
    pid_config.gyroInfluence = ConfigManager::instance().getFloat("pid/gyro_inf", pid_config.gyroInfluence);
    pid_config.headingChangeRate = ConfigManager::instance().getFloat("pid/hroc", pid_config.headingChangeRate);
    pid_config.useGyroFeedforward = ConfigManager::instance().getBool("pid/gyro_ff", pid_config.useGyroFeedforward);

    test_value_ = ConfigManager::instance().getInt("vdc/cmd", test_value_);
    test_delay_ = ConfigManager::instance().getInt("vdc/delay", test_delay_);
    test_repeat_ = ConfigManager::instance().getInt("vdc/repeat", test_repeat_);

    if (pid_config.kP != config_.pid_config.kP ||
        pid_config.kI != config_.pid_config.kI ||
        pid_config.kD != config_.pid_config.kD) {
        ESP_LOGI(TAG, "PID parameters updated: P=%.2f, I=%.2f, D=%.2f",
                pid_config.kP, pid_config.kI, pid_config.kD);
        }

    config_.pid_config = pid_config;
    steering_pid_.updateConfig(pid_config);
}

void VehicleDynamicsController::controllerTask(void* arg) {
    auto* controller = static_cast<VehicleDynamicsController*>(arg);
    TickType_t last_wake_time = xTaskGetTickCount();

    const VehicleData &vehicle_data = VehicleData::instance();

    constexpr auto ch_throttle = static_cast<size_t>(sensor::SbusChannel::THROTTLE);
    constexpr auto ch_steering = static_cast<size_t>(sensor::SbusChannel::STEERING);
    constexpr auto toggle_pidloop = static_cast<size_t>(sensor::SbusChannel::AUX7);
    constexpr auto arm_switch = static_cast<size_t>(sensor::SbusChannel::AUX8);
    constexpr auto ch_debug = static_cast<size_t>(sensor::SbusChannel::AUX9);

    while(true) {
        const float deltaTime = static_cast<float>(controller->config_.task_period) / 1000.0f;

        const sensor::SbusData& sbus_data = vehicle_data.getSbus();
        const sensor::ImuData& imu_data = vehicle_data.getImu();

        if (!sbus_data.quality.valid_signal) {
            ESP_LOGW(TAG, "Invalid SBUS signal");
            controller->esc_driver_.set_all_throttles(1000);
            // controller->esc_driver_.failsafe();
            controller->steering_servo_.setPosition(1500);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (sbus_data.channels[toggle_pidloop] > 1900 && !controller->use_pidloop_) {
            controller->enablePID(true);
        } else if (sbus_data.channels[toggle_pidloop] < 1900 && controller->use_pidloop_) {
            controller->enablePID(false);
        }

        if (!controller->use_pidloop_) {                            // Direct control mode
            controller->updateSteering(sbus_data.channels[ch_steering]);
        } else {                                                    // PID assisted control mode
            float steering_output = controller->steering_pid_.update(sbus_data, imu_data, deltaTime);
            // controller->updateSteering(steering_output);
        }

        if (sbus_data.channels[ch_debug] > 1900) {
            controller->esc_driver_.debug(
                controller->test_value_,
                controller->test_delay_,
                controller->test_repeat_
            );
            vTaskDelay(pdMS_TO_TICKS(350));
        }


        if (sbus_data.channels[arm_switch] > 1900 && !controller->armed_) {
            controller->armed_ = true;
            ESP_LOGI(TAG, "Motors armed");
        } else if (sbus_data.channels[arm_switch] < 1900 && controller->armed_) {
            controller->armed_ = false;
            // controller->esc_driver_.failsafe(); // TODO: failsafe just disables the rmt channel and there isnt a way to re-enable it when running so that needs to be rewritten
            ESP_LOGI(TAG, "Motors disarmed");
            controller->esc_driver_.set_all_throttles(1000);
        }

        if (controller->armed_) {
            controller->updateThrottle(sbus_data.channels[ch_throttle]);
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

