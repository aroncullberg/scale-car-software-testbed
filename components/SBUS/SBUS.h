#pragma once

#include <cstdint>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "data_types.h"
#include "sensor_types.h"
// #include "system_types.h"
// #include "telemetry_types.h"

#define MIN 0
#define MAX 1
#define OFFSET 2

namespace sensor {

class SBUS {
    public:
    struct Config {
        uart_port_t uart_num;
        gpio_num_t uart_tx_pin{GPIO_NUM_NC};
        gpio_num_t uart_rx_pin{GPIO_NUM_NC};
        int baud_rate{100000}; // NOTE: This value needs to be doublechecked against sbus standard baud rate, but it works so...

        uint16_t frequency{30};
    };

    explicit SBUS(const Config& config);
    ~SBUS();

    SBUS(const SBUS&) = delete;
    SBUS& operator=(const SBUS&) = delete;

    esp_err_t init() const;
    esp_err_t start();
    esp_err_t stop();

private:
    esp_err_t configureUART() const;
    
    void processFrame(const uint8_t* frame, size_t len);
    void monitorSignalQuality(); 
    static uint16_t scaleChannelValue(uint16_t raw_value, uint8_t ch);

    static void sbusTask(void* parameters);
    TaskHandle_t task_handle_{nullptr};

    Config config_t;
    SbusData current_data_{};

    static constexpr size_t FRAME_SIZE = 25;
    uint8_t frame_buffer_[FRAME_SIZE]{};
    size_t buffer_index_{0};

    bool is_running{false};

    static constexpr const char* TAG = "SBUS";
};

} // namespace sensor