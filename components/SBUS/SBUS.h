#pragma once

#include <cstdint>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "data_types.h"
#include "sensor_types.h"
// #include "system_types.h"
// #include "telemetry_types.h"

namespace sensor {

struct SbusChannelConfig {
    SbusChannelType type;
    uint16_t min_raw;
    uint16_t center_raw;  // NOTE: Only used for SYMMETRIC
    uint16_t max_raw;
};


class SBUS {
    public:
    struct Config {
        uart_port_t uart_num;
        gpio_num_t uart_tx_pin{GPIO_NUM_NC};
        gpio_num_t uart_rx_pin{GPIO_NUM_NC};
        int baud_rate{100000}; // NOTE: This value needs to be doublechecked against sbus standard baud rate. 
    };

    explicit SBUS(const Config& config);
    ~SBUS();

    // Delete copy operations - UART resource can't be shared
    SBUS(const SBUS&) = delete;
    SBUS& operator=(const SBUS&) = delete;

    esp_err_t init();
    esp_err_t start();
    esp_err_t stop();

    static const char *getChannelName(SbusChannel channel);

private:
    esp_err_t configureUART();
    
    void processFrame(const uint8_t* frame, size_t len);
    void updateChannelValues(const uint16_t* raw_channels);
    void monitorSignalQuality(); 
    float scaleChannelValue(uint16_t raw_value, const SbusChannelConfig& config);

    // Task related
    static void sbusTask(void* parameters);
    TaskHandle_t task_handle_{nullptr};

    Config config_t;
    SbusData current_data_{};

    static constexpr size_t FRAME_SIZE = 25;
    uint8_t frame_buffer_[FRAME_SIZE]{};
    size_t buffer_index_{0};

    bool is_running{false};
};

} // namespace sensor