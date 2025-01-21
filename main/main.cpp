#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sbus.h"
#include "motor_control.h"
#include "esp_log.h"

#ifndef TAG
#define TAG "main"
#endif

#define SBUS_RX_PIN GPIO_NUM_18
#define SBUS_TX_PIN GPIO_NUM_5
#define SBUS_UART UART_NUM_1
#define DSHOT_PIN GPIO_NUM_21  // Adjust based on your setup

// Global objects
SBUS *sbus = nullptr;

void sbus_read_task(void *pvParameters) {
    SBUS *sbus = (SBUS *)pvParameters;
    uint16_t channels[16];
    char log_buffer[128]; // Pre-allocate buffer for logging

    while (1) {
        if (sbus->read(channels)) {
            // Log all channels in a single message to reduce stack usage
            snprintf(log_buffer, sizeof(log_buffer), 
                    "CH: %4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d",
                    channels[0], channels[1], channels[2], channels[3],
                    channels[4], channels[5], channels[6], channels[7],
                    channels[8], channels[9], channels[10], channels[11],
                    channels[12], channels[13], channels[14], channels[15]);
            ESP_LOGI(TAG, "%s", log_buffer);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Increased delay slightly
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting SBUS example");

    // Initialize SBUS
    sbus = new SBUS(SBUS_UART);
    esp_err_t ret = sbus->begin(SBUS_RX_PIN, SBUS_TX_PIN, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SBUS");
        return;
    }

    // Increased stack size to 4096
    xTaskCreate(sbus_read_task, "sbus_read", 4096, sbus, 5, NULL);

    ESP_LOGI(TAG, "SBUS initialization complete");
}