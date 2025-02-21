#pragma once

#include <cstdint>
#include <map>

#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"
#include <esp_log.h>

class EscDriver {
public:
    enum class MotorPosition {
        FRONT_RIGHT,
        FRONT_LEFT,
        REAR_LEFT,
        REAR_RIGHT
    };

    enum class DshotCommand {                        // Source: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
        MOTOR_STOP = 0,                              // Currently not implemented
        BEEP1 = 1,                                   // Wait at least length of beep (260ms) before next command
        BEEP2 = 2,                                   // Wait at least length of beep (260ms) before next command
        BEEP3 = 3,                                   // Wait at least length of beep (260ms) before next command
        BEEP4 = 4,                                   // Wait at least length of beep (260ms) before next command
        BEEP5 = 5,                                   // Wait at least length of beep (260ms) before next command
        ESC_INFO = 6,                                // Wait at least 12ms before next command
        SPIN_DIRECTION_1 = 7,                        // Need 6x
        SPIN_DIRECTION_2 = 8,                        // Need 6x
        MODE_3D_OFF = 9,                             // Need 6x
        MODE_3D_ON = 10,                             // Need 6x
        SETTINGS_REQUEST = 11,                       // Currently not implemented
        SAVE_SETTINGS = 12,                          // Need 6x, wait at least 35ms before next command
        EXTENDED_TELEMETRY_ENABLE = 13,              // Need 6x (only on EDT enabled firmware)
        EXTENDED_TELEMETRY_DISABLE = 14,             // Need 6x (only on EDT enabled firmware)
        
        // 15-19 not yet assigned
        
        SPIN_DIRECTION_NORMAL = 20,                  // Need 6x
        SPIN_DIRECTION_REVERSED = 21,                // Need 6x
        LED0_ON = 22,
        LED1_ON = 23,
        LED2_ON = 24,
        LED3_ON = 25,
        LED0_OFF = 26,
        LED1_OFF = 27,
        LED2_OFF = 28,
        LED3_OFF = 29,
        AUDIO_STREAM_MODE_TOGGLE = 30,               // Currently not implemented
        SILENT_MODE_TOGGLE = 31,                     // Currently not implemented
        SIGNAL_LINE_TELEMETRY_DISABLE = 32,          // Need 6x. Disables commands 42 to 47
        SIGNAL_LINE_TELEMETRY_ENABLE = 33,           // Need 6x. Enables commands 42 to 47
        SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 34,  // Need 6x. Enables commands 42 to 47 and sends erpm if normal Dshot frame
        SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY = 35, // Need 6x. Enables commands 42 to 47 and sends erpm period if normal Dshot frame
        
        // 36-41 not yet assigned
        
        SIGNAL_LINE_TEMPERATURE_TELEMETRY = 42,      // 1°C per LSB
        SIGNAL_LINE_VOLTAGE_TELEMETRY = 43,          // 10mV per LSB, 40.95V max
        SIGNAL_LINE_CURRENT_TELEMETRY = 44,          // 100mA per LSB, 409.5A max
        SIGNAL_LINE_CONSUMPTION_TELEMETRY = 45,      // 10mAh per LSB, 40.95Ah max
        SIGNAL_LINE_ERPM_TELEMETRY = 46,             // 100erpm per LSB, 409500erpm max
        SIGNAL_LINE_ERPM_PERIOD_TELEMETRY = 47       // 16us per LSB, 65520us max TBD
    };


    struct Config {
        uint32_t resolution_hz{40000000};   // 40MHz resolution (ESP32-S3 typical)
        uint32_t baud_rate{300000};         // DSHOT300
        uint32_t post_delay_us{50};         // Post-frame delay
        std::map<MotorPosition, gpio_num_t> motor_pins;
        size_t mem_block_symbols{48};       // WARNING: If this is >48 then we cannot have 4 ESCs
        Config() = default;
    };

    EscDriver() = default;
    ~EscDriver();

    esp_err_t init(const Config& config);
    esp_err_t start();
    esp_err_t set_throttle(MotorPosition position, uint16_t throttle, bool telemetry = false);
    esp_err_t set_all_throttles(uint16_t throttle, bool telemetry = false);
    esp_err_t set_command(MotorPosition position, DshotCommand command, bool telemetry = false);
    esp_err_t set_all_commands(DshotCommand command, bool telemetry = false);


    bool is_initialized() const { return initialized_; }
    bool is_started() const { return started_; }
    bool is_armed() const { return armed_; }
    
    private:
    struct MotorControl {
        rmt_channel_handle_t channel = nullptr;
        rmt_encoder_handle_t encoder = nullptr;
        uint16_t current_throttle = 0;
    };
    
    static constexpr const char* TAG = "ESC Driver";
    
    Config config_;
    std::map<MotorPosition, MotorControl> motors_;
    bool initialized_ = false;
    bool started_ = false;
    bool armed_ = false;
    
    esp_err_t create_rmt_channel(gpio_num_t gpio_num, rmt_channel_handle_t* channel);
    esp_err_t create_encoder(rmt_encoder_handle_t* encoder);
    esp_err_t arm_all();
};
