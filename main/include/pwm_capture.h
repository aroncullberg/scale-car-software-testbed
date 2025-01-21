#pragma once

#include "driver/mcpwm_cap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Structure for PWM data
typedef struct {
    uint8_t channel;
    uint32_t pulse_width;
    int64_t timestamp;
} pwm_data_t;

// Structure for capture channel data
typedef struct {
    mcpwm_cap_channel_handle_t cap_chan;
    uint32_t last_capture;
    uint32_t pulse_width;
    QueueHandle_t data_queue;
} pwm_capture_t;

// External declarations
extern QueueHandle_t pwm_queue;
extern pwm_capture_t capture_channels[8];

// Function declarations
esp_err_t init_pwm_capture(uint8_t num_channels, const int *gpio_pins);
void pwm_processing_task(void *pvParameters);