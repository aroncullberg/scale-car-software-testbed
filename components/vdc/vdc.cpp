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
    pid_steering_.reset();
    pid_steering_.setKd(ConfigManager::instance().getFloat("steerpid/kd", pid_steering_.getKd()));
    pid_steering_.setKi(ConfigManager::instance().getFloat("steerpid/ki", pid_steering_.getKi()));
    pid_steering_.setKp(ConfigManager::instance().getFloat("steerpid/kp", pid_steering_.getKp()));
    pid_steering_.setAntiWindupLimit(ConfigManager::instance().getFloat("steerpid/antiwu", pid_steering_.getAntiWindupLimit()));

    // pid_motor_fl_.setKp(ConfigManager::instance().getFloat("motorpid/kp", pid_motor_fl_.getKp()));
    // pid_motor_fr_.setKp(ConfigManager::instance().getFloat("motorpid/kp", pid_motor_fr_.getKp()));
    // pid_motor_rl_.setKp(ConfigManager::instance().getFloat("motorpid/kp", pid_motor_rl_.getKp()));
    // pid_motor_rr_.setKp(ConfigManager::instance().getFloat("motorpid/kp", pid_motor_rr_.getKp()));

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

    // ret = xTaskCreatePinnedToCore(
    //     motorTask,
    //     "motor_task",
    //     4096,
    //     this,
    //     5,
    //     &motortask_handle_,
    //     1
    // );
    //
    // ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG, "Failed to create motor task");

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

    // controller->max_turn_rate_ = 120.0f;

    // ESP_LOGI(TAG, "Test kp: %f", controller->pid_steering_.getKp());
    // ESP_LOGI(TAG, "Test ki: %f", controller->pid_steering_.getKi());
    // ESP_LOGI(TAG, "Test kd: %f", controller->pid_steering_.getKd());
    // ESP_LOGI(TAG, "Test antiwu: %f", controller->pid_steering_.getAntiWindupLimit());
    // ESP_LOGI(TAG, "Test gyro deadband: %f", controller->gyro_deadband_);
    // ESP_LOGI(TAG, "Test max turn rate: %f", controller->max_turn_rate_);

    const VehicleData& vehicle_data = VehicleData::instance();
    constexpr auto ch_steering = static_cast<size_t>(sensor::SbusChannel::STEERING);
    while (true) {
        const sensor::SbusData& sbus_data = vehicle_data.getSbus();
        const sensor::ImuData& imu_data = vehicle_data.getImu();
        const sensor::channel_t steering_value = sbus_data.channels_scaled[ch_steering];

        if (controller->failsafe_) {
            controller->steering_servo_.setPosition(sensor::Servo::FAILSAFE_POSITION);
            // ESP_LOGI("TAG", "Failsafe engaged, setting steering to failsafe position");
            // vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));
            vTaskDelay(pdMS_TO_TICKS(controller->config_.frequency));

            continue;
        }

        if (controller->bypass_pid_) {
            controller->steering_servo_.setPosition(steering_value);
            // vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));
            // ESP_LOGI("TAG", "Bypassed steering engaged to %d", steering_value);
            vTaskDelay(pdMS_TO_TICKS(10));

            // static TickType_t prev_wake = 0;
            // TickType_t now = xTaskGetTickCount();
            // if (prev_wake != 0) {
            //     TickType_t delta_ticks = now - prev_wake;
            //     if (delta_ticks > 0) {
            //         uint32_t freq_hz = 1000 / delta_ticks; // assuming ticks are in ms
            //         ESP_LOGI("STEERING LOOP", "Frequency: %lu Hz", freq_hz);
            //     }
            // }
            // prev_wake = now;

            continue;
        }
        // TODO: CHeck for valid gyro data

        const float desired_rate = controller->max_turn_rate_ * (static_cast<float>(steering_value)-1000.0f) / 1000.0f;
        const float current_rate = imu_data.gyro_z * sensor::ImuData::GYRO_TO_DPS;



        float error = desired_rate - current_rate;

        // if (std::fabs(error) < controller->gyro_deadband_) {
        //     error = 0;
        // }

        const float pid_output = controller->pid_steering_.update(error);

        int16_t output = sensor::Servo::NEUTRAL_POSITION + static_cast<int16_t>(pid_output);

        // ESP_LOGI(TAG, "Desired rate: %+4f, Current rate: %+4f, PID Output %+4f", desired_rate, current_rate, pid_output);

        output = std::ranges::clamp(output, sensor::Servo::MIN_POSITION, sensor::Servo::MAX_POSITION);

        // ESP_LOGI(TAG, "steer loop: %d\n", output);

        controller->steering_servo_.setPosition(output);

        // vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));
        vTaskDelay(pdMS_TO_TICKS(10));

            // static TickType_t prev_wake = 0;
            // TickType_t now = xTaskGetTickCount();
            // if (prev_wake != 0) {
            //     TickType_t delta_ticks = now - prev_wake;
            //     if (delta_ticks > 0) {
            //         uint32_t freq_hz = 1000 / delta_ticks; // assuming ticks are in ms
            //         ESP_LOGI("STEERING LOOP", "Frequency: %lu Hz", freq_hz);
            //     }
            // }
            // prev_wake = now;


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
        const sensor::channel_t throttle_value = sbus_data.channels_scaled[ch_throttle];

        if (!sbus_data.quality.valid_signal) {
            if (!controller->failsafe_) {
                controller->failsafe_ = true;
            }
            if (!controller->flag_){
                ESP_LOGW(TAG, "No RX, entering failsafe");
                controller->flag_ = true;
            }
            // vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));
            vTaskDelay(pdMS_TO_TICKS(controller->config_.frequency));

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

        if (sbus_data.channels_scaled[ch_calibrate] > 1900) {
            ESP_LOGI(TAG, "Calibrating IMU");
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

                // vTaskDelay(pdMS_TO_TICKS(10));
                vTaskDelay(pdMS_TO_TICKS(controller->config_.frequency));
            }

            if (sample_count < 10) {
                ESP_LOGW(TAG, "Calibration failed, not enough samples");
                continue;
            }

            ESP_LOGI(TAG, "Calibration complete: Gyro bias set to x=%d, y=%d, z=%d",
                     static_cast<int16_t>(sum_gyro_x / sample_count),
                     static_cast<int16_t>(sum_gyro_y / sample_count),
                     static_cast<int16_t>(sum_gyro_z / sample_count));
            ESP_LOGI(TAG, "Calibration complete: Accel bias set to x=%d, y=%d, z=%d",
                     static_cast<int16_t>(sum_accel_x / sample_count),
                     static_cast<int16_t>(sum_accel_y / sample_count),
                     static_cast<int16_t>(sum_accel_z / sample_count));
            controller->failsafe_ = false;
        }

        if (controller->armed_) {
            const uint16_t dshot_value = std::ranges::clamp(throttle_value + 48, 48, 2047);

            // ESP_LOGI(TAG, "Throttle: %d", throttle_value);

            controller->motor_fl_.sendThrottle(dshot_value);
            controller->motor_fr_.sendThrottle(dshot_value);
            controller->motor_rl_.sendThrottle(dshot_value);
            controller->motor_rr_.sendThrottle(dshot_value);
        }

        // vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(controller->config_.frequency));
        vTaskDelay(pdMS_TO_TICKS(controller->config_.frequency));


            // static TickType_t prev_wake = 0;
            // TickType_t now = xTaskGetTickCount();
            // if (prev_wake != 0) {
            //     TickType_t delta_ticks = now - prev_wake;
            //     if (delta_ticks > 0) {
            //         uint32_t freq_hz = 1000 / delta_ticks; // assuming ticks are in ms
            //         ESP_LOGI("MAIN LOOP", "Frequency: %lu Hz", freq_hz);
            //     }
            // }
            // prev_wake = now;

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


