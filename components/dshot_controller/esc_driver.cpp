// esc_driver.cpp
#include "esc_driver.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "config_manager.h"


EscDriver::~EscDriver() {
    for(auto& [position, motor] : motors_) {
        if(motor.channel) {
            rmt_disable(motor.channel);
            rmt_del_channel(motor.channel);
        }
        if(motor.encoder) {
            rmt_del_encoder(motor.encoder);
        }
    }
}


esp_err_t EscDriver::init(const Config& config) {
    ESP_RETURN_ON_FALSE(!initialized_, ESP_ERR_INVALID_STATE, TAG, "Already initialized");
    ESP_RETURN_ON_FALSE(config.motor_pins.size() <= 4, ESP_ERR_INVALID_ARG, TAG, "Max 4 ESCs supported");


    callback_ = [this] { this->updateFromConfig(); };
    ConfigManager::instance().registerCallback(callback_);
    updateFromConfig();

    config_ = config;

    for (const auto& [position, pin] : config.motor_pins) {
        MotorControl& motor = motors_[position];

        ESP_RETURN_ON_ERROR(
            create_rmt_channel(pin, &motor.channel),
            TAG, "Failed to create RMT channel for motor position %d", static_cast<int>(position)
        );

        ESP_RETURN_ON_ERROR(
            create_encoder(&motor.encoder),
            TAG, "Failed to create encoder for motor position %d", static_cast<int>(position)
        );
        ESP_RETURN_ON_ERROR(
            rmt_enable(motor.channel),
            TAG, "Failed to enable RMT channel for motor position %d", static_cast<int>(position)
        );
    }

    initialized_ = true;
    ESP_LOGI(TAG, "esc_driver intialized sucessfully");

    return ESP_OK;
}

void EscDriver::updateFromConfig() {
    ESP_LOGI(TAG, "Updating escdriver configuration from ConfigManager");

    uint16_t new_throttle_limit = ConfigManager::instance().getInt("esc/tlimit", throttle_limit_);
    if (new_throttle_limit != throttle_limit_ && new_throttle_limit > 0 && new_throttle_limit >=100 &&  new_throttle_limit <= 2000) {
        ESP_LOGI(TAG, "Throttle limit changed: %d -> %d", throttle_limit_, new_throttle_limit);
        throttle_limit_ = new_throttle_limit;
    }

}


esp_err_t EscDriver::create_rmt_channel(gpio_num_t gpio_num, rmt_channel_handle_t* channel) const {
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

esp_err_t EscDriver::create_encoder(rmt_encoder_handle_t* encoder) const {
    dshot_esc_encoder_config_t encoder_config = {
        .resolution = config_.resolution_hz,
        .baud_rate = config_.baud_rate,
        .post_delay_us = config_.post_delay_us
    };
    
    return rmt_new_dshot_esc_encoder(&encoder_config, encoder);
}

// esp_err_t EscDriver::start() {
//     ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG, "Not initialized");
//     ESP_RETURN_ON_FALSE(!started_, ESP_ERR_INVALID_STATE, TAG, "Already started");

//     // ESP_RETURN_ON_ERROR(arm_all(), TAG, "Failed to arm motors during initializtion");

//     started_ = true;
//     return ESP_OK;
// }

esp_err_t EscDriver::arm_all() {

    for (auto& [position, motor] : motors_) {
        ESP_LOGI(TAG, "Arming: %d", static_cast<uint16_t>(position));
        // set_command(position, EscDriver::DshotCommand::MOTOR_STOP, false);
        dshot_esc_throttle_t frame = {
            .throttle = 48,
            .telemetry_req = false
        };

        rmt_transmit_config_t  transmit_config = {
            .loop_count = -1,
            .flags = {
                .eot_level = 0,
                .queue_nonblocking = false
            }
        };

        ESP_RETURN_ON_ERROR(rmt_disable(motor.channel), TAG, "Failed to disable channel");
        ESP_RETURN_ON_ERROR(rmt_enable(motor.channel), TAG, "Failed to re-enable channel");

        ESP_RETURN_ON_ERROR(
            rmt_transmit(motor.channel, motor.encoder, &frame, sizeof(frame), &transmit_config),
            TAG, "Failed to send arm signal to motor position %d", static_cast<int>(position)
        );
    }

    vTaskDelay(pdMS_TO_TICKS(350));

    // armed_ = true;
    ESP_LOGI(TAG, "All motors armed");

    return ESP_OK;
}

esp_err_t EscDriver::failsafe() {
    for (auto& [position, motor] : motors_) {
        ESP_RETURN_ON_ERROR(rmt_disable(motor.channel), TAG, "Failed to disable channel");
    }
    return ESP_OK;
}

esp_err_t EscDriver::debug(const int cmd, const int delay, const int repeat) {
    for (auto& [position, motor] : motors_) {
        ESP_LOGI(TAG, "Sending debug command %d to motor position %d", cmd, static_cast<int>(position));
        dshot_esc_throttle_t frame = {
            .throttle = static_cast<uint16_t>(cmd),
            .telemetry_req = false
        };

        rmt_transmit_config_t  transmit_config = {
            .loop_count = repeat,
            .flags = {
                .eot_level = 0,
                .queue_nonblocking = false
            }
        };

        ESP_RETURN_ON_ERROR(rmt_disable(motor.channel), TAG, "Failed to disable channel");
        ESP_RETURN_ON_ERROR(rmt_enable(motor.channel), TAG, "Failed to re-enable channel");

        ESP_RETURN_ON_ERROR(
            rmt_transmit(motor.channel, motor.encoder, &frame, sizeof(frame), &transmit_config),
            TAG, "Failed to send arm signal to motor position %d", static_cast<int>(position)
        );
    }

    ESP_LOGI(TAG, "Debug command %d sent to all motors", cmd);

    vTaskDelay(pdMS_TO_TICKS(delay));

    return ESP_OK;
}


esp_err_t EscDriver::set_throttle(MotorPosition position, sensor::channel_t input_throttle, bool telemetry) {
    ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG, "Driver not initialized or started");
    // ESP_RETURN_ON_FALSE(armed_, ESP_ERR_INVALID_STATE, TAG, "Motors not armed");
    ESP_RETURN_ON_FALSE(input_throttle <= 2000, ESP_ERR_INVALID_ARG,
        TAG, "Input throttle %hu outside valid range (0-2000)", input_throttle);

    const auto it = motors_.find(position);
    ESP_RETURN_ON_FALSE(it != motors_.end(), ESP_ERR_NOT_FOUND, TAG, "Motor position not found");
    

    const uint16_t dshot_throttle = std::clamp(input_throttle + 48, 48, throttle_limit_ * 1); // i dont know why but i need *1 here for it to not be stupid

    // if (xTaskGetTickCount() % 99 == 0) {
    //     // ESP_LOGI(TAG, "Throttle value before scaling %d", input_throttle);
    //     ESP_LOGI(TAG, "Throttle value after scaling %d", dshot_throttle);
    // }


    // TODO: Evalaute if we should use structured bindings here
    MotorControl& motor = it->second;
    const dshot_esc_throttle_t frame = {
        .throttle = dshot_throttle,
        .telemetry_req = telemetry
    };

    constexpr rmt_transmit_config_t transmit_config = {
        .loop_count = -1,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = false
        }
    };

    // Reset channel to ensure immediate throttle update
    ESP_RETURN_ON_ERROR(rmt_disable(motor.channel), TAG, "Failed to disable channel");
    ESP_RETURN_ON_ERROR(rmt_enable(motor.channel), TAG, "Failed to re-enable channel");

    const esp_err_t ret = rmt_transmit(motor.channel, motor.encoder, &frame, sizeof(frame), &transmit_config);
    if(ret == ESP_OK) {
        motor.current_throttle = dshot_throttle;
    }
    return ret;
}

esp_err_t EscDriver::set_command(MotorPosition position, DshotCommand command, bool telemetry) {
    ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG, "Driver not initialized or started");
    // ESP_RETURN_ON_FALSE(initialized_ && started_, ESP_ERR_INVALID_STATE, TAG, "Driver not initialized or started");
    // ESP_RETURN_ON_FALSE(armed_, ESP_ERR_INVALID_STATE, TAG, "Motors not armed");
    
    auto it = motors_.find(position);
    ESP_RETURN_ON_FALSE(it != motors_.end(), ESP_ERR_NOT_FOUND, TAG, "Motor position not found");
    
    MotorControl& motor = it->second;
    dshot_esc_throttle_t frame = {
        .throttle = static_cast<uint16_t>(command),
        .telemetry_req = telemetry
    };

    rmt_transmit_config_t transmit_config = {
        .loop_count = 1,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = false
        }
    };

    // Reset channel to ensure immediate command update
    ESP_RETURN_ON_ERROR(rmt_disable(motor.channel), TAG, "Failed to disable channel");
    ESP_RETURN_ON_ERROR(rmt_enable(motor.channel), TAG, "Failed to re-enable channel");

    return rmt_transmit(motor.channel, motor.encoder, &frame, sizeof(frame), &transmit_config);
}

esp_err_t EscDriver::set_all_commands(DshotCommand command, bool telemetry) {
    for(const auto& [position, _] : motors_) {
        ESP_RETURN_ON_ERROR(
            set_command(position, command, telemetry),
            TAG, "Failed to set command for motor position %d", static_cast<int>(position)
        );
    }
    return ESP_OK;
}


esp_err_t EscDriver::set_all_throttles(sensor::channel_t throttle, bool telemetry) {
    for(const auto& [position, _] : motors_) {
        ESP_RETURN_ON_ERROR(
            set_throttle(position, throttle, telemetry),
            TAG, "Failed to set throttle for motor position %d", static_cast<int>(position)
        );
    }
    return ESP_OK;
}
