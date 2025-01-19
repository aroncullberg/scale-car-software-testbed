/**
 * @file pwm_input.c
 * @brief PWM input capture implementation using ESP32 timer
 */

#include "pwm_input.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char* TAG = "pwm_input";

/** @brief Internal state for each PWM input channel */
typedef struct {
    gpio_num_t gpio_num;
    uint32_t min_pulse_us;
    uint32_t max_pulse_us;
    uint32_t timeout_ms;
    volatile uint32_t last_rise_time;
    volatile uint32_t last_fall_time;
    volatile uint32_t current_pulse_width;
    volatile bool signal_valid;
    volatile uint64_t last_valid_pulse_time;
} pwm_input_state_t;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    pwm_input_state_t* state = (pwm_input_state_t*)arg;
    uint32_t gpio_level = gpio_get_level(state->gpio_num);
    uint32_t current_time = (uint32_t)esp_timer_get_time();

    if (gpio_level) {
        // Rising edge
        state->last_rise_time = current_time;
    } else {
        // Falling edge
        state->last_fall_time = current_time;
        uint32_t pulse_width = state->last_fall_time - state->last_rise_time;

        // Validate pulse width
        if (pulse_width >= state->min_pulse_us && pulse_width <= state->max_pulse_us) {
            state->current_pulse_width = pulse_width;
            state->signal_valid = true;
            state->last_valid_pulse_time = esp_timer_get_time();
        }
    }
}

esp_err_t pwm_input_init(const pwm_input_config_t* config, pwm_input_handle_t* handle) {
    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Allocate and initialize state
    pwm_input_state_t* state = calloc(1, sizeof(pwm_input_state_t));
    if (!state) {
        return ESP_ERR_NO_MEM;
    }

    state->gpio_num = config->gpio_num;
    state->min_pulse_us = config->min_pulse_us;
    state->max_pulse_us = config->max_pulse_us;
    state->timeout_ms = config->timeout_ms;
    state->signal_valid = false;
    state->last_valid_pulse_time = 0;

    // Configure GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << config->gpio_num),
        .pull_up_en = config->pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        free(state);
        return ret;
    }

    // Install GPIO ISR service if not already installed
    static bool isr_service_installed = false;
    if (!isr_service_installed) {
        ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // ESP_ERR_INVALID_STATE means already installed
            free(state);
            return ret;
        }
        isr_service_installed = true;
    }

    // Add ISR handler
    ret = gpio_isr_handler_add(config->gpio_num, gpio_isr_handler, state);
    if (ret != ESP_OK) {
        free(state);
        return ret;
    }

    *handle = state;
    return ESP_OK;
}

esp_err_t pwm_input_deinit(pwm_input_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    pwm_input_state_t* state = (pwm_input_state_t*)handle;
    gpio_isr_handler_remove(state->gpio_num);
    free(state);
    return ESP_OK;
}

esp_err_t pwm_input_get_pulse(pwm_input_handle_t handle, uint32_t* pulse_width_us) {
    if (!handle || !pulse_width_us) {
        return ESP_ERR_INVALID_ARG;
    }

    pwm_input_state_t* state = (pwm_input_state_t*)handle;
    
    if (!pwm_input_is_valid(handle)) {
        return ESP_ERR_INVALID_STATE;
    }

    *pulse_width_us = state->current_pulse_width;
    return ESP_OK;
}

bool pwm_input_is_valid(pwm_input_handle_t handle) {
    if (!handle) {
        return false;
    }

    pwm_input_state_t* state = (pwm_input_state_t*)handle;
    uint64_t current_time = esp_timer_get_time();
    uint64_t time_since_last_pulse = (current_time - state->last_valid_pulse_time) / 1000;  // Convert to ms

    return state->signal_valid && (time_since_last_pulse <= state->timeout_ms);
}

uint32_t pwm_input_get_last_pulse_age_ms(pwm_input_handle_t handle) {
    if (!handle) {
        return UINT32_MAX;
    }

    pwm_input_state_t* state = (pwm_input_state_t*)handle;
    uint64_t current_time = esp_timer_get_time();
    return (uint32_t)((current_time - state->last_valid_pulse_time) / 1000);  // Convert to ms
}