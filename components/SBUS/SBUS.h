#pragma once

#include <cstdint>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"

namespace sensor {

enum class SbusChannelType : uint8_t {
    SYMMETRIC = 0,      // -1 to 1 (steering)
    UNIPOLAR = 1,       // 0 to 1 (throttle)
    BINARY = 2          // Switch
};

enum class SbusChannel : uint8_t {
    STEERING = 0,
    THROTTLE = 1,
    AUX1 = 2,
    AUX2 = 3,
    AUX3 = 4,
    AUX4 = 5,
    AUX5 = 6,
    AUX6 = 7,
    AUX7 = 8,
    AUX8 = 9,
    AUX9 = 10,
    AUX10 = 11,
    AUX11 = 12,
    AUX12 = 13,
    AUX13 = 14,
    AUX14 = 15,
    CHANNEL_COUNT = 16
};

struct SbusChannelConfig {
    SbusChannelType type;
    uint16_t min_raw;
    uint16_t center_raw;  // Only used for SYMMETRIC
    uint16_t max_raw;
};

// Quality metrics structure
struct SbusQuality {
    uint8_t frame_loss_percent;
    uint32_t error_count;
    float frame_interval_ms;
    bool valid_signal;
};

// Main SBUS data structure
struct SbusData {
    float channels[16];
    struct SbusQuality quality;
};

class SBUS {
    public:
    struct Config {
        uart_port_t uart_num;
        gpio_num_t uart_tx_pin;
        gpio_num_t uart_rx_pin;
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