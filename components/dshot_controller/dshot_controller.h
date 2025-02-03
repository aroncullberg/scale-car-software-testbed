// dshot_controller.h
#pragma once

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
// TODO: I realized that the dshot_esc_encoder.h already has extern "c" guards so this can be removed
extern "C" {
    #include "dshot_esc_encoder.h"
}
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "esp_check.h"


class DshotController {
public:
    // Get singleton instance
    static DshotController& instance() {
        static DshotController instance;
        return instance;
    }

    // Delete copy and move operations
    DshotController(const DshotController&) = delete;
    DshotController& operator=(const DshotController&) = delete;
    DshotController(DshotController&&) = delete;
    DshotController& operator=(DshotController&&) = delete;

    struct Config {
        struct MotorConfig {
            gpio_num_t gpio_num{GPIO_NUM_NC};
            const char* name{"undefined"};  // For logging/debugging
        };

        MotorConfig motors[4];
        uint32_t resolution_hz{40000000};  // 40MHz resolution by default
        uint32_t baud_rate{300000};        // DSHOT300 by default
        size_t mem_block_symbols{64};      // RMT memory block size
        uint32_t post_delay_us{50};        // Delay between frames
    };

    // Initialize the controller with configuration
    esp_err_t init(const Config& config);

    // Arm all motors
    esp_err_t armAll();

    // Set throttle for a specific motor (0-3)
    esp_err_t setThrottle(uint8_t motor_idx, float throttle);

    // Set throttle for all motors
    esp_err_t setThrottleAll(float throttle);

    // Send command to specific motor
    esp_err_t sendCommand(uint8_t motor_idx, uint16_t command);

    // Send command to all motors
    esp_err_t sendCommandAll(uint16_t command);

    // Stop specific motor
    esp_err_t stop(uint8_t motor_idx);

    // Stop all motors
    esp_err_t stopAll();

private:
    DshotController() = default;  // Private constructor for singleton
    ~DshotController();

    static constexpr const char* TAG = "DshotController";
    static constexpr uint8_t MOTOR_COUNT = 4;
    static constexpr uint16_t MIN_THROTTLE = 48;
    static constexpr uint16_t MAX_THROTTLE = 2047;
    static constexpr uint16_t MAX_COMMAND = 47;

    struct Motor {
        rmt_channel_handle_t channel{nullptr};
        rmt_encoder_handle_t encoder{nullptr};
        const char* name{"undefined"};
        bool initialized{false};
    };

    esp_err_t initMotor(uint8_t idx, const Config::MotorConfig& config);
    esp_err_t enableOutput(uint8_t motor_idx);
    esp_err_t disableOutput(uint8_t motor_idx);
    esp_err_t transmitValue(uint8_t motor_idx, uint16_t value, bool telemetry_req = false);
    bool isValidMotorIndex(uint8_t idx) const { return idx < MOTOR_COUNT; }

    Config config_{};
    Motor motors_[MOTOR_COUNT];
    bool initialized_{false};
};