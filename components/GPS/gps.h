#pragma once

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "TinyGPS++.h"

#include "sensor_types.h"

#define MATCH 0

namespace sensor {

class GPS {
public:
    struct Config {
        uart_port_t uart_num{UART_NUM_1};
        gpio_num_t uart_tx_pin{GPIO_NUM_NC};
        gpio_num_t uart_rx_pin{GPIO_NUM_NC};
        int baud_rate{9600};
        size_t rx_buffer_size{2048};
        size_t tx_buffer_size{0};
        TickType_t task_period {pdMS_TO_TICKS(100)};
    };

    GPS(const Config& config);
    ~GPS();

    esp_err_t init();
    esp_err_t start();
    esp_err_t stop();

private:
    static constexpr const char* TAG = "GPS";

    void processGPSData();  // New method to handle data updates

    esp_err_t configureUART();

    static void gpsTask(void* parameters);
    TaskHandle_t task_handle_{nullptr};

    TinyGPSPlus tiny_gps_;
    Config config_t;
    GpsData current_data{}; // TODO: rename to current_data_

    bool is_running{false};
};

}