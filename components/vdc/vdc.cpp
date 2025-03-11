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
    constexpr auto ch_arm = static_cast<size_t>(sensor::SbusChannel::AUX10);

    while(true) {
        const float deltaTime = static_cast<float>(controller->config_.task_period) / 1000.0f;

        const sensor::SbusData& sbus_data = vehicle_data.getSbus();
        const sensor::ImuData& imu_data = vehicle_data.getImu();

        if (!sbus_data.quality.valid_signal) {
            ESP_LOGW(TAG, "Invalid SBUS signal");
            controller->steering_servo_.setPosition(sensor::Servo::FAILSAFE_POSITION);
            controller->esc_driver_.set_all_throttles(sensor::Motor::FAILSAFE_THROTTLE); // TODO: Change this to be a realfailsafe where the rmt driver shuts off so esc just shuts down (i hope they do atelast)
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (sbus_data.channels_scaled[toggle_pidloop] > 1900 && !controller->use_pidloop_) {
            ESP_LOGI(TAG, "PID control enabled");
            controller->enablePID(true);
        } else if (sbus_data.channels_scaled[toggle_pidloop] < 1900 && controller->use_pidloop_) {
            ESP_LOGI(TAG, "PID control disabled");
            controller->enablePID(false);
        }

        if (!controller->use_pidloop_) {                            // Direct control mode
            controller->updateSteering(sbus_data.channels_scaled[ch_steering]);
        } else {                                                    // PID assisted control mode
            // const sensor::channel_t steering_output = controller->steering_pid_.update(sbus_data, imu_data, deltaTime);
            // controller->updateSteering(steering_output);
         }

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

esp_err_t VehicleDynamicsController::updateSteering(sensor::channel_t steering_value) {
    return steering_servo_.setPosition(steering_value);
}

