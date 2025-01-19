/**
 * @file pwm_input.h
 * @brief PWM input capture for RC receiver signals using ESP32 timer
 */

#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum number of supported PWM input channels */
#define PWM_INPUT_MAX_CHANNELS 8

/** @brief PWM input configuration structure */
typedef struct {
    gpio_num_t gpio_num;          /*!< GPIO number for input capture */
    uint32_t min_pulse_us;        /*!< Minimum valid pulse width in microseconds */
    uint32_t max_pulse_us;        /*!< Maximum valid pulse width in microseconds */
    uint32_t timeout_ms;          /*!< Timeout for signal loss detection in milliseconds */
    bool pull_up;                 /*!< Enable internal pull-up resistor */
    bool invert_input;            /*!< Invert input signal */
} pwm_input_config_t;

/** @brief PWM input channel handle */
typedef void* pwm_input_handle_t;

/**
 * @brief Initialize PWM input capture on specified GPIO
 * @param config Pointer to PWM input configuration
 * @param handle Pointer to store the created handle
 * @return ESP_OK on success
 */
esp_err_t pwm_input_init(const pwm_input_config_t* config, pwm_input_handle_t* handle);

/**
 * @brief Deinitialize PWM input capture
 * @param handle PWM input handle to destroy
 * @return ESP_OK on success
 */
esp_err_t pwm_input_deinit(pwm_input_handle_t handle);

/**
 * @brief Get latest valid pulse width reading
 * @param handle PWM input handle
 * @param pulse_width_us Pointer to store pulse width in microseconds
 * @return ESP_OK if valid reading available, ESP_ERR_INVALID_STATE if signal lost
 */
esp_err_t pwm_input_get_pulse(pwm_input_handle_t handle, uint32_t* pulse_width_us);

/**
 * @brief Check if PWM signal is currently valid
 * @param handle PWM input handle
 * @return true if signal is valid, false if signal lost
 */
bool pwm_input_is_valid(pwm_input_handle_t handle);

/**
 * @brief Get time since last valid pulse
 * @param handle PWM input handle
 * @return Time in milliseconds since last valid pulse
 */
uint32_t pwm_input_get_last_pulse_age_ms(pwm_input_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* PWM_INPUT_H */