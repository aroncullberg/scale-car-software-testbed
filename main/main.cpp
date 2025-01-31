#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "data_pool.h"

#ifndef TAG
#define TAG "main"
#endif

void control_task(void *params) {
    const TickType_t xDelay = pdMS_TO_TICKS(100);

    while(1) {
        sensor::SbusData sbus = VehicleData::instance().getSbus();
        uint32_t timestamp = VehicleData::instance().getSbusTimestamp();
        uint32_t current_time = xTaskGetTickCount();

        bool is_fresh = (current_time - timestamp) < pdMS_TO_TICKS(150);

        if (is_fresh && sbus.quality.valid_signal) {
            // Log header with quality metrics
            ESP_LOGI(TAG, "SBUS Channels (frame loss: %d%%, Interval: %.2fms)",
                        sbus.quality.frame_loss_percent,
                        sbus.quality.frame_interval_ms);

            // Single macro call with variable arguments
            ESP_LOGI(TAG, 
                "Steering: %2.4f, Throttle: %2.4f, AUX1: %2.4f, AUX2: %2.4f, AUX3: %2.4f, "
                "AUX4: %2.4f, AUX5: %2.4f, AUX6: %2.4f, AUX7: %2.4f, AUX8: %2.4f, "
                "AUX9: %2.4f, AUX10: %2.4f, AUX11: %2.4f, AUX12: %2.4f, AUX13: %2.4f, AUX14: %2.4f",
                sbus.channels[0], sbus.channels[1], sbus.channels[2], sbus.channels[3],
                sbus.channels[4], sbus.channels[5], sbus.channels[6], sbus.channels[7],
                sbus.channels[8], sbus.channels[9], sbus.channels[10], sbus.channels[11],
                sbus.channels[12], sbus.channels[13], sbus.channels[14], sbus.channels[15]);
        } else {
            ESP_LOGW(TAG, "SBUS data stale or invalid! Signal valid: %d, Fresh %d",
                        sbus.quality.valid_signal, is_fresh);
        }
        vTaskDelay(xDelay);
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting SBUS test application");

    sensor::SBUS::Config sbus_config = {
        .uart_num = UART_NUM_1,        // Using UART1
        .uart_tx_pin = GPIO_NUM_17,    // Adjust according to your wiring
        .uart_rx_pin = GPIO_NUM_18,    // Adjust according to your wiring
        .baud_rate = 100000            // SBUS runs at 100k baud
    };

    // Create SBUS instance
    sensor::SBUS sbus(sbus_config);

    // Initialize SBUS
    esp_err_t err = sbus.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SBUS: %d", err);
        return;
    }

    // Start SBUS processing
    err = sbus.start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start SBUS: %d", err);
        return;
    }

    BaseType_t task_created = xTaskCreate(
       control_task,       // Task function
       "control_task",     // Task name
       4096,              // Stack size (in words)
       NULL,              // Task parameters (NULL since not needed)
       tskIDLE_PRIORITY + 1, // Task priority (higher than idle)
       NULL               // Task handle (NULL since not used)
    );


    if (task_created != pdPASS) {
       ESP_LOGE(TAG, "Failed to create SBUS task");
       return;
    }

    // Keep the main task alive
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}