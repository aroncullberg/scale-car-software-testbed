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
      steering_servo_(config.servo_config),
      motor_fr_(config.motors_config.front_right_pin, config.motors_config.dshot_mode),
      motor_fl_(config.motors_config.front_left_pin, config.motors_config.dshot_mode),
      motor_rl_(config.motors_config.rear_left_pin, config.motors_config.dshot_mode),
      motor_rr_(config.motors_config.rear_right_pin, config.motors_config.dshot_mode) {

    callback_ = [this] { this->updateFromConfig(); };
    ConfigManager::instance().registerCallback(callback_);
    updateFromConfig();
}

VehicleDynamicsController::~VehicleDynamicsController() {
    stop();
}

esp_err_t VehicleDynamicsController::init() {
    esp_err_t err = steering_servo_.setPosition(sensor::Servo::NEUTRAL_POSITION);
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to initialize steering servo");

    // TODO: Initialize pid with config values at init
    pid_steering_.setKd(ConfigManager::instance().getFloat("steerpid/kd", pid_steering_.getKd()));
    pid_steering_.setKi(ConfigManager::instance().getFloat("steerpid/ki", pid_steering_.getKi()));
    pid_steering_.setKp(ConfigManager::instance().getFloat("steerpid/kp", pid_steering_.getKp()));
    pid_steering_.setAntiWindupLimit(ConfigManager::instance().getFloat("steerpid/antiwu", pid_steering_.getAntiWindupLimit()));

    pid_motor_fl_.setKp(ConfigManager::instance().getFloat("motorpid/kp", pid_motor_fl_.getKp()));
    pid_motor_fr_.setKp(ConfigManager::instance().getFloat("motorpid/kp", pid_motor_fr_.getKp()));
    pid_motor_rl_.setKp(ConfigManager::instance().getFloat("motorpid/kp", pid_motor_rl_.getKp()));
    pid_motor_rr_.setKp(ConfigManager::instance().getFloat("motorpid/kp", pid_motor_rr_.getKp()));

    motor_fr_.begin_UNSAFE();
    motor_fl_.begin_UNSAFE();
    motor_rl_.begin_UNSAFE();
    motor_rr_.begin_UNSAFE();

    ESP_LOGI(TAG, "Arming all motors (300ms)");
    vTaskDelay(pdMS_TO_TICKS(500));


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

    ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG, "Failed to vdc task");

    ret = xTaskCreatePinnedToCore(
        steeringTask,
        "steering_task",
        4096,
        this,
        5,
        &steeringtask_handle_,
        1
    );

    ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG, "Failed to create steering task");

    ret = xTaskCreatePinnedToCore(
        motorTask,
        "motor_task",
        4096,
        this,
        5,
        &motortask_handle_,
        1
    );

    ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG, "Failed to create motor task");

    is_running_ = true;
    return ESP_OK;
}

esp_err_t VehicleDynamicsController::stop() {
    if (!is_running_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Clean up task
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    is_running_ = false;
    return ESP_OK;
}

void VehicleDynamicsController::updateFromConfig() {
    ESP_LOGI(TAG, "Updating VDC configuration from ConfigManager");



    float new_steerpid_kp = ConfigManager::instance().getFloat("steerpid/kp", pid_steering_.getKp());
    if (new_steerpid_kp != pid_steering_.getKp()) {
        ESP_LOGI(TAG, "Steering PID Kp changed: %f -> %f",
                 pid_steering_.getKp(),
                 new_steerpid_kp);
        pid_steering_.setKp(new_steerpid_kp);
    }

    float new_steerpid_ki = ConfigManager::instance().getFloat("steerpid/ki", pid_steering_.getKi());
    if (new_steerpid_ki != pid_steering_.getKi()) {
        ESP_LOGI(TAG, "Steering PID Ki changed: %f -> %f",
                 pid_steering_.getKi(),
                 new_steerpid_ki);
        pid_steering_.setKi(new_steerpid_ki);
    }

    float new_steerpid_kd = ConfigManager::instance().getFloat("steerpid/kd", pid_steering_.getKd());
    if (new_steerpid_kd != pid_steering_.getKd()) {
        ESP_LOGI(TAG, "Steering PID Kd changed: %f -> %f",
                 pid_steering_.getKd(),
                 new_steerpid_kd);
        pid_steering_.setKd(new_steerpid_kd);
    }

    float new_steerpid_antiwu = ConfigManager::instance().getFloat("steerpid/antiwu", pid_steering_.getAntiWindupLimit());
    if (new_steerpid_antiwu != pid_steering_.getAntiWindupLimit()) {
        ESP_LOGI(TAG, "Steering PID Anti-Windup changed: %f -> %f",
                 pid_steering_.getAntiWindupLimit(),
                 new_steerpid_antiwu);
        pid_steering_.setAntiWindupLimit(new_steerpid_antiwu);
    }

    float new_gyro_deadband = ConfigManager::instance().getFloat("vdc/deadband", gyro_deadband_);
    if (new_gyro_deadband != gyro_deadband_) {
        ESP_LOGI(TAG, "Gyro deadband changed: %f -> %f",
                 gyro_deadband_,
                 new_gyro_deadband);
        gyro_deadband_ = new_gyro_deadband;
    }

    float new_motorpid_kp = ConfigManager::instance().getFloat("motorpid/kp", pid_motor_fl_.getKp());
    if (new_motorpid_kp != pid_motor_fl_.getKp()) {
        ESP_LOGI(TAG, "Motor PID Kp changed: %f -> %f",
                 pid_motor_fl_.getKp(),
                 new_motorpid_kp);
        pid_motor_fl_.setKp(new_motorpid_kp);
        pid_motor_fr_.setKp(new_motorpid_kp);
        pid_motor_rl_.setKp(new_motorpid_kp);
        pid_motor_rr_.setKp(new_motorpid_kp);
    }
    float new_motorpid_ki = ConfigManager::instance().getFloat("motorpid/ki", pid_motor_fl_.getKi());
    if (new_motorpid_ki != pid_motor_fl_.getKi()) {
        ESP_LOGI(TAG, "Motor PID Ki changed: %f -> %f",
                 pid_motor_fl_.getKi(),
                 new_motorpid_ki);
        pid_motor_fl_.setKi(new_motorpid_ki);
        pid_motor_fr_.setKi(new_motorpid_ki);
        pid_motor_rl_.setKi(new_motorpid_ki);
        pid_motor_rr_.setKi(new_motorpid_ki);
    }

    float new_motorpid_kd = ConfigManager::instance().getFloat("motorpid/kd", pid_motor_fl_.getKd());
    if (new_motorpid_kd != pid_motor_fl_.getKd()) {
        ESP_LOGI(TAG, "Motor PID Kd changed: %f -> %f",
                 pid_motor_fl_.getKd(),
                 new_motorpid_kd);
        pid_motor_fl_.setKd(new_motorpid_kd);
        pid_motor_fr_.setKd(new_motorpid_kd);
        pid_motor_rl_.setKd(new_motorpid_kd);
        pid_motor_rr_.setKd(new_motorpid_kd);
    }

    float new_motorpid_antiwu = ConfigManager::instance().getFloat("motorpid/antiwu", pid_motor_fl_.getAntiWindupLimit());
    if (new_motorpid_antiwu != pid_motor_fl_.getAntiWindupLimit()) {
        ESP_LOGI(TAG, "Motor PID Anti-Windup changed: %f -> %f",
                 pid_motor_fl_.getAntiWindupLimit(),
                 new_motorpid_antiwu);
        pid_motor_fl_.setAntiWindupLimit(new_motorpid_antiwu);
        pid_motor_fr_.setAntiWindupLimit(new_motorpid_antiwu);
        pid_motor_rl_.setAntiWindupLimit(new_motorpid_antiwu);
        pid_motor_rr_.setAntiWindupLimit(new_motorpid_antiwu);
    }

    bool new_log_erp = ConfigManager::instance().getBool("vdc/logerpm", log_erp_);
    if (new_log_erp != log_erp_) {
        ESP_LOGI(TAG, "Log eRPM changed: %s -> %s",
                 log_erp_ ? "true" : "false",
                 new_log_erp ? "true" : "false");
        log_erp_ = new_log_erp;
    }

    float new_turnrate = ConfigManager::instance().getFloat("vdc/turnrate", max_turn_rate_);
    if (new_turnrate != max_turn_rate_) {
        ESP_LOGI(TAG, "Turnrate scale changed %f -> %f",
            max_turn_rate_, new_turnrate);
        max_turn_rate_ = new_turnrate;
    }
}

void VehicleDynamicsController::steeringTask(void* arg) {
    auto* controller = static_cast<VehicleDynamicsController*>(arg);
    TickType_t last_wake_time = xTaskGetTickCount();

    controller->pid_steering_.enablePID();

    const VehicleData& vehicle_data = VehicleData::instance();
    constexpr auto ch_steering = static_cast<size_t>(sensor::SbusChannel::STEERING);
    while (true) {
        const sensor::SbusData& sbus_data = vehicle_data.getSbus();
        const sensor::ImuData& imu_data = vehicle_data.getImu();
        const sensor::channel_t steering_value = sbus_data.channels_scaled[ch_steering];

        if (controller->failsafe_) {
            controller->steering_servo_.setPosition(sensor::Servo::FAILSAFE_POSITION);
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));
            continue;
        }

        if (controller->bypass_pid_) {
            controller->steering_servo_.setPosition(steering_value);
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));
            continue;
        }
        // TODO: CHeck for valid gyro data

        const float desired_rate = controller->max_turn_rate_ * (static_cast<float>(steering_value)-1000.0f) / 1000.0f;
        const float current_rate = imu_data.gyro_z * sensor::ImuData::GYRO_TO_DPS;

        float error = desired_rate - current_rate;

        if (std::fabs(error) < controller->gyro_deadband_) {
            error = 0;
        }

        const float pid_output = controller->pid_steering_.update(error);

        int16_t output = sensor::Servo::NEUTRAL_POSITION + static_cast<int16_t>(pid_output);

        output = std::ranges::clamp(output, sensor::Servo::MIN_POSITION, sensor::Servo::MAX_POSITION);

        controller->steering_servo_.setPosition(output);

        // TODO: implement fixed frequency stuff

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));
    }
}

void VehicleDynamicsController::motorTask(void* arg) {
    auto* controller = static_cast<VehicleDynamicsController*>(arg);
    TickType_t last_wake_time = xTaskGetTickCount();


    const VehicleData& vehicle_data = VehicleData::instance();
    constexpr auto ch_throttle = static_cast<size_t>(sensor::SbusChannel::THROTTLE);

    while (true) {
        const sensor::SbusData& sbus_data = vehicle_data.getSbus();
        const sensor::eRPMData& erpm_data = vehicle_data.getErpm();
        const sensor::channel_t throttle_value = sbus_data.channels_scaled[ch_throttle];


        const uint16_t dshot_value = std::ranges::clamp(throttle_value + 48, 48, 2047);

        if (!controller->armed_) {
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));
            continue;
        }

        controller->motor_fl_.sendThrottle(dshot_value);
        controller->motor_fr_.sendThrottle(dshot_value);
        controller->motor_rl_.sendThrottle(dshot_value);
        controller->motor_rr_.sendThrottle(dshot_value);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));

        continue;

        // NOTE: i realized there is a fundamental logic error in this setup so i dont think we can do it like this.

        if (throttle_value < 100 || controller->bypass_pid_) {
            controller->motor_fl_.sendThrottle(dshot_value);
            controller->motor_fr_.sendThrottle(dshot_value);
            controller->motor_rl_.sendThrottle(dshot_value);
            controller->motor_rr_.sendThrottle(dshot_value);

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));
            continue;
        }

        int32_t error_fl{0};
        int32_t error_fr{0};
        int32_t error_rl{0};
        int32_t error_rr{0};


        if (erpm_data.front_left.erpm != 0 && erpm_data.front_left.erpm != 65535) {
            error_fl = throttle_value * controller->throttle_to_rpm_throttle_ - erpm_data.front_left.erpm;
            // int32_t output_fl =  dshot_value + controller->pid_motor_fl_.update(error_fl);
            // controller->motor_fl_.sendThrottle(output_fl);
            controller->motor_fl_.sendThrottle(dshot_value);
        } else {
            controller->motor_fl_.sendThrottle(dshot_value);
        }

        if (erpm_data.front_right.erpm != 0 && erpm_data.front_right.erpm != 65535) {
            error_fr = throttle_value * controller->throttle_to_rpm_throttle_ - erpm_data.front_right.erpm;
            // int32_t output_fr = dshot_value + controller->pid_motor_fr_.update(error_fr);
            // controller->motor_fr_.sendThrottle(output_fr);
            controller->motor_fr_.sendThrottle(dshot_value);
        } else {
            controller->motor_fr_.sendThrottle(dshot_value);
        }

        if (erpm_data.rear_left.erpm != 0 && erpm_data.rear_left.erpm != 65535) {
            error_rl = throttle_value * controller->throttle_to_rpm_throttle_ - erpm_data.rear_left.erpm;
            // int32_t output_rl = dshot_value + controller->pid_motor_rl_.update(error_rl);
            // controller->motor_rl_.sendThrottle(output_rl);
            controller->motor_rl_.sendThrottle(dshot_value);
        } else {
            controller->motor_rl_.sendThrottle(dshot_value);
        }

        if (erpm_data.rear_right.erpm != 0 && erpm_data.rear_right.erpm != 65535) {
            error_rr = throttle_value * controller->throttle_to_rpm_throttle_ - erpm_data.rear_right.erpm;
            // int32_t output_rr = dshot_value + static_cast<int32_t>(controller->pid_motor_rr_.update(error_rr));
            // controller->motor_rr_.sendThrottle(output_rr);
            controller->motor_rr_.sendThrottle(dshot_value);
        } else {
            controller->motor_rr_.sendThrottle(dshot_value);
        }

        controller->updateRPMTelemetry(throttle_value);

        static int count = 0;
        if (controller->log_erp_ && count++ % 100 == 0) {
            ESP_LOGI(TAG, "ERPM: %05lu %05lu %05lu %05lu (%05lu) | ePID offset on (%05u): %05f %05f %05f %05f",
                     erpm_data.front_right.erpm,
                     erpm_data.front_left.erpm,
                     erpm_data.rear_left.erpm,
                     erpm_data.rear_right.erpm,
                     throttle_value * controller->throttle_to_rpm_throttle_,
                        dshot_value,
                        controller->pid_motor_rr_.update(error_fr),
                        controller->pid_motor_rr_.update(error_fl),
                        controller->pid_motor_rr_.update(error_rl),
                        controller->pid_motor_rr_.update(error_rr)
                     );
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));
    }

}


[[noreturn]] void VehicleDynamicsController::controllerTask(void* arg) {
    auto* controller = static_cast<VehicleDynamicsController*>(arg);
    TickType_t last_wake_time = xTaskGetTickCount();

    const VehicleData& vehicle_data = VehicleData::instance();

    constexpr auto ch_throttle = static_cast<size_t>(sensor::SbusChannel::THROTTLE);
    // constexpr auto ch_steering = static_cast<size_t>(sensor::SbusChannel::STEERING);
    constexpr auto ch_arm = static_cast<size_t>(sensor::SbusChannel::AUX1);
    constexpr auto ch_pid_bypass = static_cast<size_t>(sensor::SbusChannel::AUX2);
    constexpr auto ch_steering_kp = static_cast<size_t>(sensor::SbusChannel::AUX3);
    constexpr auto ch_steering_ki = static_cast<size_t>(sensor::SbusChannel::AUX4);
    constexpr auto ch_steering_kd = static_cast<size_t>(sensor::SbusChannel::AUX5);
    constexpr auto ch_calibrate = static_cast<size_t>(sensor::SbusChannel::AUX6);

    while (true) {
        const sensor::SbusData& sbus_data = vehicle_data.getSbus();

        if (!sbus_data.quality.valid_signal) {
            if (!controller->failsafe_) {
                controller->failsafe_ = true;
            }
            if (!controller->flag_){
                ESP_LOGW(TAG, "No RX, entering failsafe");
                controller->flag_ = true;
            }
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));
            continue;
        }
        controller->failsafe_ = false;
        controller->flag_ = true;

        if (sbus_data.channels_scaled[ch_pid_bypass] > 1900 && !controller->bypass_pid_) {
            controller->bypass_pid_ = true;
            ESP_LOGI(TAG, "Bypassing PID");
        }
        else if (sbus_data.channels_scaled[ch_pid_bypass] < 1900 && controller->bypass_pid_) {
            controller->bypass_pid_ = false;
            ESP_LOGI(TAG, "Resuming PID");
        }

        if (sbus_data.channels_scaled[ch_arm] > 1900 && !controller->armed_ && sbus_data.channels_scaled[
            ch_throttle] < 25) {
            controller->armed_ = true;
            ESP_LOGI(TAG, "Armed!");
        }
        else if (sbus_data.channels_scaled[ch_arm] < 1900 && controller->armed_) {
            controller->armed_ = false;
            ESP_LOGI(TAG, "Disarmed (or is it?)");
        }

        if (sbus_data.channels_scaled[ch_steering_kp] < 500) {
            controller->pid_steering_.setKp(controller->pid_steering_.getKp() - 0.1f);
            ESP_LOGI(TAG, "KP: %f", controller->pid_steering_.getKp());
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (sbus_data.channels_scaled[ch_steering_kp] > 1900) {
            controller->pid_steering_.setKp(controller->pid_steering_.getKp() + 0.1f);
            ESP_LOGI(TAG, "KP: %f", controller->pid_steering_.getKp());
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (sbus_data.channels_scaled[ch_steering_kd] < 500) {
            controller->pid_steering_.setKd(controller->pid_steering_.getKd() - 0.1f);
            ESP_LOGI(TAG, "KD: %f", controller->pid_steering_.getKd());
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (sbus_data.channels_scaled[ch_steering_kd] > 1900) {
            controller->pid_steering_.setKd(controller->pid_steering_.getKd() + 0.1f);
            ESP_LOGI(TAG, "KD: %f", controller->pid_steering_.getKd());
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (sbus_data.channels_scaled[ch_steering_ki] < 500) {
            controller->pid_steering_.setKi(controller->pid_steering_.getKi() - 0.1f);
            ESP_LOGI(TAG, "KI: %f", controller->pid_steering_.getKi());
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (sbus_data.channels_scaled[ch_steering_ki] > 1900) {
            controller->pid_steering_.setKi(controller->pid_steering_.getKi() + 0.1f);
            ESP_LOGI(TAG, "KI: %f", controller->pid_steering_.getKi());
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (sbus_data.channels_scaled[ch_calibrate] > 1900) {
            controller->failsafe_ = true;

            constexpr TickType_t calibrate_duration = pdMS_TO_TICKS(3000);
            TickType_t start_time = xTaskGetTickCount();

            int64_t sum_gyro_x = 0;
            int64_t sum_gyro_y = 0;
            int64_t sum_gyro_z = 0;
            int64_t sum_accel_x = 0;
            int64_t sum_accel_y = 0;
            int64_t sum_accel_z = 0;
            uint32_t sample_count = 0;

            while (xTaskGetTickCount() - start_time < calibrate_duration) {
                sum_gyro_x += vehicle_data.getImu().gyro_x;
                sum_gyro_y += vehicle_data.getImu().gyro_y;
                sum_gyro_z += vehicle_data.getImu().gyro_z;
                sum_accel_x += vehicle_data.getImu().accel_x;
                sum_accel_y += vehicle_data.getImu().accel_y;
                sum_accel_z += vehicle_data.getImu().accel_z;
                sample_count++;

                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (sample_count < 10) {
                ESP_LOGW(TAG, "Calibration failed, not enough samples");
                continue;
            }

            sensor::ImuData imu_data = vehicle_data.getImu();
            imu_data.bias.gyro.x = static_cast<int16_t>(sum_gyro_x / sample_count);
            imu_data.bias.gyro.y = static_cast<int16_t>(sum_gyro_y / sample_count);
            imu_data.bias.gyro.z = static_cast<int16_t>(sum_gyro_z / sample_count);
            imu_data.bias.accel.x = static_cast<int16_t>(sum_accel_x / sample_count);
            imu_data.bias.accel.y = static_cast<int16_t>(sum_accel_y / sample_count);
            imu_data.bias.accel.z = static_cast<int16_t>(sum_accel_z / sample_count);
            ESP_LOGI(TAG, "Calibration complete: Gyro bias set to x=%d, y=%d, z=%d",
                     imu_data.bias.gyro.x,
                     imu_data.bias.gyro.y,
                     imu_data.bias.gyro.z);
            ESP_LOGI(TAG, "Calibration complete: Accel bias set to x=%d, y=%d, z=%d",
                     imu_data.bias.accel.x,
                     imu_data.bias.accel.y,
                     imu_data.bias.accel.z);

            controller->failsafe_ = false;
        }


        // TickType_t now = xTaskGetTickCount();
        // TickType_t delta_ticks = now - last_wake_time;
        //
        // // Avoid divide-by-zero
        // if (delta_ticks > 0) {
        //     uint32_t freq_hz = 1000 / delta_ticks;
        //     ESP_LOGI(TAG, "Frequency: %u Hz", freq_hz);
        // }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));


    }
}

void VehicleDynamicsController::updateRPMTelemetry(sensor::channel_t throttle_value) {
    // Non-blocking RPM reading (could use waitForErpm for blocking reads)
    uint32_t rpm_fr = motor_fr_.getErpm();
    uint32_t rpm_fl = motor_fl_.getErpm();
    uint32_t rpm_rl = motor_rl_.getErpm();
    uint32_t rpm_rr = motor_rr_.getErpm();

    sensor::eRPMData eRPM;
    if (rpm_fr != 0 || rpm_fr != 65535) {
        eRPM.front_right.erpm = rpm_fr;
    }
    if (rpm_fl != 0 || rpm_fl != 65535) {
        eRPM.front_left.erpm = rpm_fl;

    }
    if (rpm_rl != 0 || rpm_rl != 65535) {
        eRPM.rear_left.erpm = rpm_rl;
    }
    if (rpm_rr != 0 || rpm_rr != 65535) {
        eRPM.rear_right.erpm = rpm_rr;
    }
    eRPM.throttle_value = throttle_value;

    VehicleData::instance().updateErpm(eRPM);
}


