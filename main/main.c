/**
 * @file main.c
 * @brief Example of reading multiple RC channels using PWM input module
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "pwm_input.h"
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"

#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_ESC_RESOLUTION_HZ 32000000 // 32MHz resolution, DSHot protocol needs a relative high resolution
#else
#define DSHOT_ESC_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution
#endif
#define DSHOT_ESC_GPIO_NUM      7

static const char* TAG = "rc_reader";

// RC channel configuration
#define NUM_RC_CHANNELS 2
#define RC_UPDATE_RATE_MS 20  // 50Hz update rate

// GPIO pins for RC channels (adjust as needed)
static const gpio_num_t rc_pins[NUM_RC_CHANNELS] = {
    GPIO_NUM_35,    
    GPIO_NUM_36,    
};

// Structure to hold RC channel data
typedef struct {
    pwm_input_handle_t handle;
    uint32_t pulse_width;
    bool signal_valid;
} rc_channel_t;

// Array to store RC channel states
static rc_channel_t rc_channels[NUM_RC_CHANNELS];

typedef struct {
    uint16_t throttle;
    float steering;
} rc_values_t;

rc_values_t rc_values = {
    .throttle = 0,
    .steering = 0.0f,
};

/**
 * @brief Initialize all RC input channels
 * @return ESP_OK on success
 */
static esp_err_t init_rc_channels(void) {
    esp_err_t ret = ESP_OK;

    // Common configuration for all channels
    pwm_input_config_t config = {
        .min_pulse_us = 900,    // FrSky typical minimum
        .max_pulse_us = 2100,   // FrSky typical maximum
        .timeout_ms = 500,      // Signal timeout
        .pull_up = true,        // Enable pull-up
        .invert_input = false   // Don't invert input
    };

    // Initialize each channel
    for (int i = 0; i < NUM_RC_CHANNELS; i++) {
        config.gpio_num = rc_pins[i];
        
        ret = pwm_input_init(&config, &rc_channels[i].handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize RC channel %d on GPIO %d", i + 1, rc_pins[i]);
            return ret;
        }
        
        rc_channels[i].pulse_width = 0;
        rc_channels[i].signal_valid = false;

        ESP_LOGI(TAG, "Initialized RC channel %d on GPIO %d", i + 1, rc_pins[i]);
    }

    return ret;
}

/**
 * @brief Map PWM value from input range to output range
 */
static float map_pwm_to_float(uint32_t pwm_value, float out_min, float out_max) {
    const float in_min = 1000.0f;  // Typical PWM minimum
    const float in_max = 2000.0f;  // Typical PWM maximum
    
    return (pwm_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Map PWM value from input range to output range with capped input and output
 * @param pwm_value Input PWM value to map
 * @param out_min Minimum output value
 * @param out_max Maximum output value
 * @return Mapped and capped uint16_t value
 */
static uint16_t map_pwm_to_uint16(uint32_t pwm_value, uint16_t out_min, uint16_t out_max) {
    // Constants for PWM input range
    const uint32_t in_min = 1000;  // Typical PWM minimum
    const uint32_t in_max = 2000;  // Typical PWM maximum
    
    // Cap input value to valid range
    uint32_t capped_pwm = pwm_value;
    if (capped_pwm < in_min) capped_pwm = in_min;
    if (capped_pwm > in_max) capped_pwm = in_max;
    
    // Calculate mapped value using integer arithmetic
    uint32_t mapped = (capped_pwm - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    
    // Cap output value (though it shouldn't be necessary due to capped input)
    if (mapped < out_min) mapped = out_min;
    if (mapped > out_max) mapped = out_max;
    
    return (uint16_t)mapped;
}

/**
 * @brief Task to read and process RC channels
 */
static void rc_reader_task(void* pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Update all channels
        for (int i = 0; i < NUM_RC_CHANNELS; i++) {
            uint32_t pulse_width;
            esp_err_t ret = pwm_input_get_pulse(rc_channels[i].handle, &pulse_width);
            
            rc_channels[i].signal_valid = (ret == ESP_OK);
            if (rc_channels[i].signal_valid) {
                rc_channels[i].pulse_width = pulse_width;
            }
        }

        // Log channel values if all signals are valid
        bool all_valid = true;
        for (int i = 0; i < NUM_RC_CHANNELS; i++) {
            if (!rc_channels[i].signal_valid) {
                all_valid = false;
                break;
            }
        }

        if (all_valid) {
            // Map raw PWM values to -1.0 to 1.0 range for better readability
            rc_values.steering = map_pwm_to_float(rc_channels[0].pulse_width, -1.0f, 1.0f);
            rc_values.throttle = map_pwm_to_uint16(rc_channels[1].pulse_width, 0.0f, 2047.0f);
            //float steering = map_pwm_to_float(rc_channels[0].pulse_width, -1.0f, 1.0f);
            //float throttle = map_pwm_to_float(rc_channels[1].pulse_width, 0.0f, 1000.0f);
            //float elevator = map_pwm_to_float(rc_channels[2].pulse_width, -1.0f, 1.0f);
            //float rudder = map_pwm_to_float(rc_channels[3].pulse_width, -1.0f, 1.0f);

            ESP_LOGI(TAG, "RC Values - STEER: %.2f, THROTTLE: %.2d",rc_values.steering, rc_values.throttle);
        } else {
            ESP_LOGW(TAG, "One or more RC channels have invalid signals!");
            
            // Print individual channel status
            for (int i = 0; i < NUM_RC_CHANNELS; i++) {
                if (!rc_channels[i].signal_valid) {
                    uint32_t age = pwm_input_get_last_pulse_age_ms(rc_channels[i].handle);
                    ESP_LOGW(TAG, "Channel %d signal lost for %lu ms", i + 1, age);
                }
            }
        }

        // Run at fixed update rate
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(RC_UPDATE_RATE_MS));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t esc_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
        .gpio_num = DSHOT_ESC_GPIO_NUM,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &esc_chan));

    ESP_LOGI(TAG, "Install Dshot ESC encoder");
    rmt_encoder_handle_t dshot_encoder = NULL;
    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300 protocol
        .post_delay_us = 50, // extra delay between each frame
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(esc_chan));

    rmt_transmit_config_t tx_config = {
        .loop_count = -1, // infinite loop
    };
    dshot_esc_throttle_t throttle = {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    };

    ESP_LOGI(TAG, "Start ESC by sending zero throttle for a while...");
    ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
    vTaskDelay(pdMS_TO_TICKS(5000));

    


    ESP_LOGI(TAG, "Initializing RC reader example...");

    // Initialize RC channels
    esp_err_t ret = init_rc_channels();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RC channels! Error: %d", ret);
        return;
    }

    // Create RC reader task
    BaseType_t task_created = xTaskCreate(
        rc_reader_task,         // Task function
        "rc_reader",            // Task name
        4096,                   // Stack size (bytes)
        NULL,                   // Task parameters
        5,                      // Task priority
        NULL                    // Task handle
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RC reader task!");
        return;
    }

    ESP_LOGI(TAG, "RC reader initialized successfully");
    ESP_LOGI(TAG, "Reading %d RC channels at %dHz", NUM_RC_CHANNELS, 1000/RC_UPDATE_RATE_MS);

    
    while(1) {
        throttle.throttle = rc_values.throttle;
        ESP_LOGI(TAG, "Throttle sent to esc: %.2d",throttle.throttle);
        
        //rc_values.dshot_throttle.throttle = thro;
        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
        // the previous loop transfer is till undergoing, we need to stop it and restart,
        // so that the new throttle can be updated on the output
        ESP_ERROR_CHECK(rmt_disable(esc_chan));
        ESP_ERROR_CHECK(rmt_enable(esc_chan));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}