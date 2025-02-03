#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "data_pool.h"
#include "imu.h"
#include "SBUS.h"
#include "gps.h"
#include "DataManager.h"
#include "telemetry_manager.h"

#ifndef TAG
#define TAG "main"
#endif


extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting SBUS test application");

    // SBUS >>
    sensor::SBUS::Config sbus_config = {
        .uart_num = UART_NUM_1,        // Using UART1
        .uart_tx_pin = GPIO_NUM_NC,    // Adjust according to your wiring
        .uart_rx_pin = GPIO_NUM_18,    // Adjust according to your wiring
        .baud_rate = 100000            // SBUS runs at 100k baud
    };

    sensor::SBUS sbus(sbus_config);
    esp_err_t err = sbus.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SBUS: %d", err);
        return;
    }

    err = sbus.start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start SBUS: %d", err);
        return;
    }

    // << SBUS 

    // GPS >>
    sensor::GPS::Config gps_config = {
        .uart_num = UART_NUM_2,        // Using UART1
        .uart_tx_pin = GPIO_NUM_7,    // Adjust according to your wiring
        .uart_rx_pin = GPIO_NUM_15,    // Adjust according to your wiring
        .baud_rate = 57600,            // SBUS runs at 100k baud
        .rx_buffer_size = 2048,
        .tx_buffer_size = 1024,
    };

    sensor::GPS gps(gps_config);
    err = gps.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize GPS: %d", err);
        return;
    }

    err = gps.start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start GPS: %d", err);
        return;
    }

    // << GPS 


    // IMU >>
    sensor::IMU::Config imu_config = {
        .spi_host = SPI2_HOST,
        .spi_miso_pin = 13,
        .spi_mosi_pin = 11,
        .spi_sck_pin = 12,
        .spi_cs_pin = 10,
    };

    sensor::IMU imu(imu_config);
    err = imu.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IMU: %d", err);
        return;
    }

    err = imu.start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start IMU: %d", err);
        return;
    }

    // << IMU 

    DataManager::Config dm_config = {
        .task_period = pdMS_TO_TICKS(1000)
    };

    DataManager* dm = new DataManager(dm_config);
    ESP_ERROR_CHECK(dm->start());

    // Keep the main task alive
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}