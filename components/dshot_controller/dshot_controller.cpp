// dshot_controller.cpp
#include "dshot_controller.h"
#include "esp_log.h"

DshotController::~DshotController() {
    if (initialized_) {
        stopAll();
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            if (motors_[i].initialized) {
                rmt_disable(motors_[i].channel);
                rmt_del_encoder(motors_[i].encoder);
                rmt_del_channel(motors_[i].channel);
            }
        }
    }
}

esp_err_t DshotController::init(const Config& config) {
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;

    // Initialize each motor
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        esp_err_t err = initMotor(i, config.motors[i]);
        if (err != ESP_OK) {
            // Cleanup already initialized motors
            for (uint8_t j = 0; j < i; j++) {
                if (motors_[j].initialized) {
                    rmt_disable(motors_[j].channel);
                    rmt_del_encoder(motors_[j].encoder);
                    rmt_del_channel(motors_[j].channel);
                }
            }
            return err;
        }
    }

    initialized_ = true;
    return ESP_OK;
}

esp_err_t DshotController::initMotor(uint8_t idx, const Config::MotorConfig& config) {
    if (config.gpio_num == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_ARG;
    }

    // Configure RMT TX channel
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = config.gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = config_.resolution_hz,
        .mem_block_symbols = config_.mem_block_symbols,
        .trans_queue_depth = 10,
    };
    
    esp_err_t err = rmt_new_tx_channel(&tx_chan_config, &motors_[idx].channel);
    if (err != ESP_OK) return err;

    // Configure DSHOT encoder
    dshot_esc_encoder_config_t encoder_config = {
        .resolution = config_.resolution_hz,
        .baud_rate = config_.baud_rate,
        .post_delay_us = config_.post_delay_us,
    };
    
    err = rmt_new_dshot_esc_encoder(&encoder_config, &motors_[idx].encoder);
    if (err != ESP_OK) {
        rmt_del_channel(motors_[idx].channel);
        return err;
    }

    motors_[idx].name = config.name;
    motors_[idx].initialized = true;
    return ESP_OK;
}

esp_err_t DshotController::armAll() {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Enable all outputs and send zero throttle
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        ESP_RETURN_ON_ERROR(enableOutput(i), TAG, "Motor %d: Failed to enable output", i);
        ESP_RETURN_ON_ERROR(transmitValue(i, 0), TAG, "Motor %d: Failed to transmit zero throttle", i);
    }
    
    // Wait for 5 seconds (typical arming time)
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    return ESP_OK;
}

esp_err_t DshotController::setThrottle(uint8_t motor_idx, float throttle) {
    if (!initialized_ || !isValidMotorIndex(motor_idx)) {
        return ESP_ERR_INVALID_STATE;
    }

    // Clamp throttle to valid range
    throttle = (throttle < 0.0f) ? 0.0f : ((throttle > 1.0f) ? 1.0f : throttle);

    // Convert to DSHOT value (48-2047)
    uint16_t dshot_value = MIN_THROTTLE + 
        static_cast<uint16_t>((MAX_THROTTLE - MIN_THROTTLE) * throttle);

    return transmitValue(motor_idx, dshot_value);
}

esp_err_t DshotController::setThrottleAll(float throttle) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        esp_err_t err = setThrottle(i, throttle);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t DshotController::sendCommand(uint8_t motor_idx, uint16_t command) {
    if (!initialized_ || !isValidMotorIndex(motor_idx)) {
        return ESP_ERR_INVALID_STATE;
    }

    if (command > MAX_COMMAND) {
        return ESP_ERR_INVALID_ARG;
    }

    return transmitValue(motor_idx, command);
}

esp_err_t DshotController::sendCommandAll(uint16_t command) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        esp_err_t err = sendCommand(i, command);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t DshotController::stop(uint8_t motor_idx) {
    if (!initialized_ || !isValidMotorIndex(motor_idx)) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(transmitValue(motor_idx, 0), TAG, "Failed to send zero throttle");
    return disableOutput(motor_idx);
}

esp_err_t DshotController::stopAll() {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        esp_err_t err = stop(i);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t DshotController::enableOutput(uint8_t motor_idx) {
    return rmt_enable(motors_[motor_idx].channel);
}

esp_err_t DshotController::disableOutput(uint8_t motor_idx) {
    return rmt_disable(motors_[motor_idx].channel);
}

esp_err_t DshotController::transmitValue(uint8_t motor_idx, uint16_t value, bool telemetry_req) {
    dshot_esc_throttle_t throttle = {
        .throttle = value,
        .telemetry_req = telemetry_req
    };

    rmt_transmit_config_t tx_config = {
        .loop_count = -1, // Continuous transmission
    };

    ESP_RETURN_ON_ERROR(disableOutput(motor_idx), TAG, "Failed to disable output");
    ESP_RETURN_ON_ERROR(enableOutput(motor_idx), TAG, "Failed to enable output");
    
    return rmt_transmit(motors_[motor_idx].channel, motors_[motor_idx].encoder, 
                       &throttle, sizeof(throttle), &tx_config);
}