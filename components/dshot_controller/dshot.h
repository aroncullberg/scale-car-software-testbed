#pragma once

#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "dshot_esc_encoder.h"

namespace motor {

/**
 * @brief Configuration structure for DSHOT ESC GPIOs
 */
struct DshotGpioConfig {
    gpio_num_t front_left{GPIO_NUM_NC};
    gpio_num_t front_right{GPIO_NUM_NC};
    gpio_num_t rear_left{GPIO_NUM_NC};
    gpio_num_t rear_right{GPIO_NUM_NC};
};

/**
 * @brief Configuration for DSHOT ESC driver
 */
struct DshotConfig {
    DshotGpioConfig esc_gpio;                          // GPIO configuration for each ESC
    uint32_t rmt_resolution_hz{40000000};              // 40MHz resolution by default
    uint32_t baud_rate{300000};                        // DSHOT300 by default
    uint32_t post_delay_us{50};                        // Post frame delay in microseconds
    size_t mem_block_symbols{64};                      // RMT memory block size
    size_t trans_queue_depth{10};                      // RMT transaction queue depth
};

/**
 * @brief Motor indices for individual control
 */
enum class MotorIndex : uint8_t {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3,
    COUNT = 4
};

class Dshot {
public:
    /**
     * @brief Construct a new Dshot object
     * 
     * @param config Configuration structure
     */
    explicit Dshot(const DshotConfig& config);
    
    /**
     * @brief Destroy the Dshot object and cleanup resources
     */
    ~Dshot();

    /**
     * @brief Initialize the DSHOT driver
     * 
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t init();

    /**
     * @brief Start all ESC channels
     * 
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t start();

    /**
     * @brief Stop all ESC channels
     * 
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t stop();

    /**
     * @brief Set speed for a specific motor
     * 
     * @param motor Motor index
     * @param throttle Throttle value (0-1000)
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t setMotorSpeed(MotorIndex motor, uint16_t throttle);

    /**
     * @brief Set speeds for all motors simultaneously
     * 
     * @param throttles Array of 4 throttle values (0-1000)
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t setAllMotorSpeeds(const uint16_t throttles[static_cast<size_t>(MotorIndex::COUNT)]);

private:
    static constexpr const char* TAG = "DSHOT";
    static constexpr size_t MOTOR_COUNT = static_cast<size_t>(MotorIndex::COUNT);

    // Delete copy constructor and assignment operator
    Dshot(const Dshot&) = delete;
    Dshot& operator=(const Dshot&) = delete;

    /**
     * @brief Configure RMT channel for a specific motor
     * 
     * @param index Motor index
     * @param gpio GPIO number
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t configureRmtChannel(size_t index, gpio_num_t gpio);

    /**
     * @brief Update throttle for a specific motor
     * 
     * @param channel_index RMT channel index
     * @param throttle Throttle value
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t updateThrottle(size_t channel_index, uint16_t throttle);

    // Configuration
    DshotConfig config_;

    // State tracking
    bool initialized_{false};
    bool running_{false};

    // RMT handles
    rmt_channel_handle_t esc_chan_[MOTOR_COUNT]{nullptr};
    rmt_encoder_handle_t dshot_encoder_{nullptr};

    // Current throttle values
    uint16_t current_throttles_[MOTOR_COUNT]{0};
};

} // namespace motor