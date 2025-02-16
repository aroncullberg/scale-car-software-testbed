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

namespace sensor {

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
// 
    // static const char *getChannelName(SbusChannel channel);

private:
    esp_err_t configureUART();
    
    void processFrame(const uint8_t* frame, size_t len);
    void monitorSignalQuality(); 
    uint16_t scaleChannelValue(uint16_t raw_value, uint16_t min_raw, uint16_t max_raw);

    // Task related
    static void sbusTask(void* parameters);
    TaskHandle_t task_handle_{nullptr};

    Config config_t;
    SbusData current_data_{};

    static constexpr size_t FRAME_SIZE = 25;
    uint8_t frame_buffer_[FRAME_SIZE]{};
    size_t buffer_index_{0};

    bool is_running{false};

    static constexpr char* TAG = "SBUS";

    // NOTE: Yeah this is uglu but its better than haiving a gian switchstament where this is used, i like it, it stays.
    static constexpr uint16_t CHANNEL_CONFIGS[][2] = {
        {CONFIG_SBUS_CH0_MIN, CONFIG_SBUS_CH0_MAX},
        {CONFIG_SBUS_CH1_MIN, CONFIG_SBUS_CH1_MAX},
        {CONFIG_SBUS_CH2_MIN, CONFIG_SBUS_CH2_MAX},
        {CONFIG_SBUS_CH3_MIN, CONFIG_SBUS_CH3_MAX},
        {CONFIG_SBUS_CH4_MIN, CONFIG_SBUS_CH4_MAX},
        {CONFIG_SBUS_CH5_MIN, CONFIG_SBUS_CH5_MAX},
        {CONFIG_SBUS_CH6_MIN, CONFIG_SBUS_CH6_MAX},
        {CONFIG_SBUS_CH7_MIN, CONFIG_SBUS_CH7_MAX},
        {CONFIG_SBUS_CH8_MIN, CONFIG_SBUS_CH8_MAX},
        {CONFIG_SBUS_CH9_MIN, CONFIG_SBUS_CH9_MAX},
        {CONFIG_SBUS_CH10_MIN, CONFIG_SBUS_CH10_MAX},
        {CONFIG_SBUS_CH11_MIN, CONFIG_SBUS_CH11_MAX},
        {CONFIG_SBUS_CH12_MIN, CONFIG_SBUS_CH12_MAX},
        {CONFIG_SBUS_CH13_MIN, CONFIG_SBUS_CH13_MAX},
        {CONFIG_SBUS_CH14_MIN, CONFIG_SBUS_CH14_MAX},
        {CONFIG_SBUS_CH15_MIN, CONFIG_SBUS_CH15_MAX},
    };
};

} // namespace sensor