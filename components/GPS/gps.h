#pragma once

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "TinyGPS++.h"

#define MATCH 0

namespace sensor {

struct GPSData {
    // Position data (unchanged)
    int32_t latitude{0};    // Degrees * 10^7
    int32_t longitude{0};   // Degrees * 10^7
    int32_t altitude_mm{0}; // Altitude in millimeters

    // Speed data (new)
    uint32_t speed_mmps{0};    // Speed in millimeters per second
    uint16_t ground_course{0}; // Course in centidegrees (0-36000)
    bool speed_valid{false};   // Indicates if speed data is valid

    // Quality data (unchanged)
    struct {
        uint8_t fix_type{0};      
        uint8_t satellites{0};     
        uint8_t satellites_used{0};
        uint16_t hdop{0};         
    } quality;

    // Status flags (unchanged)
    union {
        uint8_t flags{0};
        struct {
            uint8_t valid_fix : 1;    
            uint8_t north_south : 1;  
            uint8_t east_west : 1;    
            uint8_t reserved : 5;     
        } bits;
    } status;

    // UTC time (unchanged)
    struct {
        uint8_t hours{0};
        uint8_t minutes{0};
        uint8_t seconds{0};
        uint16_t milliseconds{0};
    } time;

    GPSData() = default;
};
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
    GPSData current_data{}; // TODO: rename to current_data_

    bool is_running{false};
};

}