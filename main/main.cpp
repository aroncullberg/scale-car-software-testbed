#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "data_pool.h"
#include "imu.h"
#include "SBUS.h"
#include "gps.h"
// #include "esp_netif.h"
#include "telemetry_manager.h"
#include "servo.h"
#include "esc_driver.h"
#include "nvs_flash.h"
// #include "esp_event.h"
#include "vehicle_dynamics_controller.h"
#include "esp_mac.h"

#ifndef TAG
#define TAG "main"
#endif

extern "C" void app_main(void) {
    // Configure telemetry manager
    telemetry::TelemetryManager::Config telemetry_config = {
        .peer_mac = {0xDC, 0xDA, 0x0C, 0x2A, 0x17, 0xD8},
        .queue_size = 16,
        .task_period = pdMS_TO_TICKS(10),
        .fetcher_period = pdMS_TO_TICKS(20),
        .esp_now_channel = 0,
        .task_priority = 5,
        .fetcher_priority = 5,
        .task_stack_size = 4096,
        .fetcher_stack_size = 4096
    };

    ESP_ERROR_CHECK(telemetry::TelemetryManager::instance().init(telemetry_config));
    ESP_ERROR_CHECK(telemetry::TelemetryManager::instance().start());

    if (CONFIG_SBUS_ENABLE) {
        sensor::SBUS::Config sbus_config = {
            .uart_num = static_cast<uart_port_t>(CONFIG_SBUS_UART_NUM),        
            .uart_tx_pin = GPIO_NUM_17,    
            .uart_rx_pin = static_cast<gpio_num_t>(CONFIG_SBUS_UART_RX),    
            .baud_rate = 100000            // SBUS runs at 100k baud
        };
        static sensor::SBUS sbus(sbus_config);
        ESP_ERROR_CHECK(sbus.init());
        ESP_ERROR_CHECK(sbus.start());
    }


    if (CONFIG_GPS_ENABLE) {
        sensor::GPS::Config gps_config = {
            .uart_num = static_cast<uart_port_t>(CONFIG_GPS_UART_NUM),
            .uart_tx_pin = static_cast<gpio_num_t>(CONFIG_GPS_UART_TX),
            .uart_rx_pin = static_cast<gpio_num_t>(CONFIG_GPS_UART_RX),
            .baud_rate = 57600, // NOTE: this specific one runs at 57600 even though the manual specifies the defualt is 9600 (which doens't work). Which is why i wont add to kconfig (no im not just lazy)
            .rx_buffer_size = 2048,
            .tx_buffer_size = 1024,
        };
        static sensor::GPS gps(gps_config); // WARNING: This has to be a static or its killed because out-of-scope(?) after if-statement
        ESP_ERROR_CHECK(gps.init());
        ESP_ERROR_CHECK(gps.start());
    }


    if (CONFIG_IMU_ENABLE) {
        sensor::IMU::Config imu_config = {
            .spi_host = SPI2_HOST,
            .spi_miso_pin = CONFIG_IMU_SPI_MISO,
            .spi_mosi_pin = CONFIG_IMU_SPI_MOSI,
            .spi_sck_pin = CONFIG_IMU_SPI_CLK,
            .spi_cs_pin = CONFIG_IMU_SPI_CS,
        };
        static sensor::IMU imu(imu_config);
        ESP_ERROR_CHECK(imu.init());
        ESP_ERROR_CHECK(imu.start());
    }
    
    // Configure the steering servo
    Servo::Config servo_config = {
        .gpio_num = static_cast<gpio_num_t>(CONFIG_SERVO_OUTPUT_GPIO),  
        .min_pulse_width_us = static_cast<int>(CONFIG_SERVO_MIN_PULSE_WIDTH_US),  
        .max_pulse_width_us = static_cast<int>(CONFIG_SERVO_MAX_PULSE_WIDTH_US),  
        .freq_hz = static_cast<uint32_t>(CONFIG_SERVO_FREQUENCY_HZ)  
    };

    // UPDATE: New ESC driver configuration
    EscDriver::Config esc_config;
    esc_config.motor_pins[EscDriver::MotorPosition::FRONT_RIGHT] = static_cast<gpio_num_t>(38);
    esc_config.motor_pins[EscDriver::MotorPosition::FRONT_LEFT] = static_cast<gpio_num_t>(39);
    esc_config.motor_pins[EscDriver::MotorPosition::REAR_LEFT] = static_cast<gpio_num_t>(40);
    esc_config.motor_pins[EscDriver::MotorPosition::REAR_RIGHT] = static_cast<gpio_num_t>(41);

    VehicleDynamicsController::Config vd_config = {
        .steering_servo = servo_config,
        .esc_config = esc_config,       
        .task_stack_size = 4096,
        .task_priority = 5,
        .task_period = pdMS_TO_TICKS(20)
    };
    

    VehicleDynamicsController vd_controller(vd_config);
    ESP_ERROR_CHECK(vd_controller.init());
    ESP_ERROR_CHECK(vd_controller.start());

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}