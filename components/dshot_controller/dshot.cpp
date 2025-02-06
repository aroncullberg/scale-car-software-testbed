#include "dshot.h"
#include "esp_log.h"
#include "esp_check.h"

namespace motor {

Dshot::Dshot(const DshotConfig& config) 
    : config_(config) {
}

Dshot::~Dshot() {
    if (running_) {
        stop();
    }
    
    // Clean up RMT channels
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        if (esc_chan_[i]) {
            rmt_del_channel(esc_chan_[i]);
        }
    }

    // Clean up encoder
    if (dshot_encoder_) {
        rmt_del_encoder(dshot_encoder_);
    }
}

esp_err_t Dshot::init() {
    esp_err_t ret = ESP_OK;

    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Configure RMT channels for each motor
    const gpio_num_t gpios[MOTOR_COUNT] = {
        config_.esc_gpio.front_left,
        config_.esc_gpio.front_right,
        config_.esc_gpio.rear_left,
        config_.esc_gpio.rear_right
    };

    // Check if all GPIOs are valid
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        if (gpios[i] == GPIO_NUM_NC) {
            ESP_LOGE(TAG, "GPIO not configured for motor %d", i);
            return ESP_ERR_INVALID_ARG;
        }
    }

    // Create DSHOT ESC encoder
    dshot_esc_encoder_config_t encoder_config = {
        .resolution = config_.rmt_resolution_hz,
        .baud_rate = config_.baud_rate,
        .post_delay_us = config_.post_delay_us
    };
    
    ret = rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create DSHOT encoder");
        return ret;
    }

    // Configure RMT channel for each motor
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        ret = configureRmtChannel(i, gpios[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure RMT channel for motor %d", i);
            return ret;
        }
    }

    initialized_ = true;
    return ESP_OK;
}

esp_err_t Dshot::configureRmtChannel(size_t index, gpio_num_t gpio) {
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = config_.rmt_resolution_hz,
        .mem_block_symbols = config_.mem_block_symbols,
        .trans_queue_depth = config_.trans_queue_depth,
    };

    return rmt_new_tx_channel(&tx_chan_config, &esc_chan_[index]);
}

esp_err_t Dshot::start() {
    esp_err_t ret = ESP_OK;

    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (running_) {
        return ESP_OK;
    }

    // Enable all RMT channels
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        ret = rmt_enable(esc_chan_[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable RMT channel %d", i);
            return ret;
        }
    }

    // Set initial zero throttle to all motors
    uint16_t zero_throttles[MOTOR_COUNT] = {0};
    ret = setAllMotorSpeeds(zero_throttles);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set initial zero throttle");
        return ret;
    }

    running_ = true;
    return ESP_OK;
}

esp_err_t Dshot::stop() {
    if (!running_) {
        return ESP_OK;
    }

    // Disable all RMT channels
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        esp_err_t ret = rmt_disable(esc_chan_[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to disable RMT channel %d", i);
            return ret;
        }
    }

    running_ = false;
    return ESP_OK;
}

esp_err_t Dshot::updateThrottle(size_t channel_index, uint16_t throttle) {
    if (!running_) {
        return ESP_ERR_INVALID_STATE;
    }

    dshot_esc_throttle_t dshot_throttle = {
        .throttle = throttle,
        .telemetry_req = false
    };

    rmt_transmit_config_t tx_config = {
        .loop_count = -1 // infinite loop
    };

    // Following the example's pattern of disable/enable for throttle updates
    ESP_RETURN_ON_ERROR(rmt_disable(esc_chan_[channel_index]), TAG, "Failed to disable channel");
    ESP_RETURN_ON_ERROR(rmt_transmit(esc_chan_[channel_index], dshot_encoder_, 
                                    &dshot_throttle, sizeof(dshot_throttle), 
                                    &tx_config), 
                       TAG, "Failed to transmit throttle");
    ESP_RETURN_ON_ERROR(rmt_enable(esc_chan_[channel_index]), TAG, "Failed to enable channel");

    current_throttles_[channel_index] = throttle;
    return ESP_OK;
}

esp_err_t Dshot::setMotorSpeed(MotorIndex motor, uint16_t throttle) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t index = static_cast<size_t>(motor);
    if (index >= MOTOR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    if (throttle > 1000) {
        ESP_LOGW(TAG, "Throttle value %d exceeds maximum (1000), clamping", throttle);
        throttle = 1000;
    }

    return updateThrottle(index, throttle);
}

esp_err_t Dshot::setAllMotorSpeeds(const uint16_t throttles[MOTOR_COUNT]) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_OK;

    // Update all motors
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        uint16_t throttle = throttles[i];
        if (throttle > 1000) {
            ESP_LOGW(TAG, "Throttle value %d for motor %d exceeds maximum (1000), clamping", 
                    throttle, i);
            throttle = 1000;
        }

        ret = updateThrottle(i, throttle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update throttle for motor %d", i);
            return ret;
        }
    }

    return ESP_OK;
}

} // namespace motor