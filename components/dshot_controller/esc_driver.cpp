// esc_driver.cpp
#include "esc_driver.h"
#include "esp_log.h"
#include "esp_check.h"

static const char* TAG = "ESC Driver";

EscDriver::~EscDriver() {
    for(auto& motor : motors_) {
        if(motor.channel) {
            rmt_disable(motor.channel);
            rmt_del_channel(motor.channel);
        }
        if(motor.encoder) {
            rmt_del_encoder(motor.encoder);
        }
    }
}

esp_err_t EscDriver::initialize(const Config& config) {
    ESP_RETURN_ON_FALSE(!initialized_, ESP_ERR_INVALID_STATE, TAG, "Already initialized");
    ESP_RETURN_ON_FALSE(config.gpio_nums.size() <= 4, ESP_ERR_INVALID_ARG, TAG, "Max 4 ESCs supported");

    config_ = config;
    motors_.resize(config.gpio_nums.size());

    for(size_t i = 0; i < config.gpio_nums.size(); ++i) {
        ESP_RETURN_ON_ERROR(
            create_rmt_channel(config.gpio_nums[i], &motors_[i].channel),
            TAG, "Failed to create RMT channel for ESC %zu", i
        );
        
        ESP_RETURN_ON_ERROR(
            create_encoder(&motors_[i].encoder),
            TAG, "Failed to create encoder for ESC %zu", i
        );
    }

    initialized_ = true;
    return ESP_OK;
}

esp_err_t EscDriver::create_rmt_channel(gpio_num_t gpio_num, rmt_channel_handle_t* channel) {
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = config_.resolution_hz,
        .mem_block_symbols = config_.mem_block_symbols,
        .trans_queue_depth = 4,
        .flags = {
            .invert_out = false,
            .with_dma = false,
            .io_loop_back = false,
            .io_od_mode = false
        }
    };

    return rmt_new_tx_channel(&tx_config, channel);
}

esp_err_t EscDriver::create_encoder(rmt_encoder_handle_t* encoder) {
    dshot_esc_encoder_config_t encoder_config = {
        .resolution = config_.resolution_hz,
        .baud_rate = config_.baud_rate,
        .post_delay_us = config_.post_delay_us
    };
    
    return rmt_new_dshot_esc_encoder(&encoder_config, encoder);
}

esp_err_t EscDriver::start() {
    ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG, "Not initialized");
    ESP_RETURN_ON_FALSE(!started_, ESP_ERR_INVALID_STATE, TAG, "Already started");

    for(size_t i = 0; i < motors_.size(); ++i) {
        ESP_RETURN_ON_ERROR(
            rmt_enable(motors_[i].channel),
            TAG, "Failed to enable RMT channel %zu", i
        );
    }

    started_ = true;
    return ESP_OK;
}

esp_err_t EscDriver::arm_motor(size_t motor_idx, bool arm) {
    ESP_RETURN_ON_FALSE(motor_idx < motors_.size(), ESP_ERR_INVALID_ARG, TAG, "Invalid motor index");
    
    MotorControl& motor = motors_[motor_idx];
    if(arm && !motor.armed) {
        // Arm sequence: send zero throttle for initialization
        ESP_RETURN_ON_ERROR(
            set_throttle(motor_idx, 0),
            TAG, "Failed arm sequence for motor %zu", motor_idx
        );
        motor.armed = true;
        ESP_LOGI(TAG, "Motor %zu armed", motor_idx);
    } else if(!arm && motor.armed) {
        // Disarm sequence: send zero throttle and mark disarmed
        ESP_RETURN_ON_ERROR(
            set_throttle(motor_idx, 0),
            TAG, "Failed disarm sequence for motor %zu", motor_idx
        );
        motor.armed = false;
        ESP_LOGI(TAG, "Motor %zu disarmed", motor_idx);
    }
    return ESP_OK;
}

esp_err_t EscDriver::set_throttle(size_t motor_idx, uint16_t throttle, bool telemetry) {
    ESP_RETURN_ON_FALSE(motor_idx < motors_.size(), ESP_ERR_INVALID_ARG, TAG, "Invalid motor index");
    ESP_RETURN_ON_FALSE(throttle <= 1999, ESP_ERR_INVALID_ARG, TAG, "Throttle %hu exceeds DShot maximum (1999)", throttle);
    
    MotorControl& motor = motors_[motor_idx];
    ESP_RETURN_ON_FALSE(throttle == 0 || motor.armed, 
                      ESP_ERR_INVALID_STATE, TAG, 
                      "Cannot set throttle %hu - motor %zu not armed", throttle, motor_idx);

    dshot_esc_throttle_t frame = {
        .throttle = throttle,
        .telemetry_req = telemetry
    };

    rmt_transmit_config_t transmit_config = {
        .loop_count = -1,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = false
        }
    };

    // Reset channel to ensure immediate throttle update
    ESP_RETURN_ON_ERROR(rmt_disable(motor.channel), TAG, "Failed to disable channel %zu", motor_idx);
    ESP_RETURN_ON_ERROR(rmt_enable(motor.channel), TAG, "Failed to re-enable channel %zu", motor_idx);

    esp_err_t ret = rmt_transmit(motor.channel, motor.encoder, &frame, sizeof(frame), &transmit_config);
    if(ret == ESP_OK) {
        motor.current_throttle = throttle;
    }
    return ret;
}
