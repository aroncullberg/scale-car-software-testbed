#pragma once

#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"

/**
 * @brief Voltage sensor using ADC with configurable voltage divider
 *
 * Singleton class that runs a background FreeRTOS task for continuous sampling.
 * Can optionally send CRSF battery telemetry automatically.
 * Thread-safe read access from anywhere in the code.
 *
 * Supports voltage divider networks for reading voltages higher than 3.3V.
 * Formula: Vin = Vout * (R1 + R2) / R2
 * Where:
 * - Vin = Input voltage (battery voltage)
 * - Vout = ADC reading voltage
 * - R1 = Resistor from Vin to ADC pin (higher side)
 * - R2 = Resistor from ADC pin to GND (lower side)
 *
 * Example: To measure 0-16.5V with 3.3V ADC:
 * - Use R1 = 100kΩ, R2 = 33kΩ
 * - Ratio = (100k + 33k) / 33k = 4.03
 * - Max input = 3.3V * 4.03 = 13.3V (safe for 4S LiPo)
 */

class VoltageSensor {
public:
    struct Config {
        adc_channel_t adc_channel;      // ADC channel (e.g., ADC_CHANNEL_0)
        adc_atten_t attenuation;        // ADC attenuation (e.g., ADC_ATTEN_DB_11 for 0-3.3V)
        uint32_t resistor_r1_ohms;      // R1 - High-side resistor (from Vin to ADC pin)
        uint32_t resistor_r2_ohms;      // R2 - Low-side resistor (from ADC pin to GND)
        uint32_t samples_per_read;      // Number of samples to average per reading (default: 16)
        uint32_t sample_interval_ms;    // Time between readings (default: 100ms)

        // Telemetry settings
        bool enable_telemetry;          // Enable automatic CRSF battery telemetry transmission
        uint32_t telemetry_interval_ms; // Interval for telemetry transmission (default: 1000ms)
        uint32_t battery_capacity_mah;  // Battery capacity in mAh for telemetry
    };

    static VoltageSensor& instance();

    // Delete copy operations
    VoltageSensor(const VoltageSensor&) = delete;
    VoltageSensor& operator=(const VoltageSensor&) = delete;

    /**
     * @brief Initialize the ADC and calibration
     * @param config Configuration parameters
     * @return ESP_OK on success
     */
    esp_err_t init(const Config& config);

    /**
     * @brief Start the background sampling task
     * @return ESP_OK on success
     */
    esp_err_t start();

    /**
     * @brief Stop the background sampling task
     */
    void stop();

    /**
     * @brief Get latest voltage reading in millivolts (thread-safe)
     * @return Voltage in mV (after voltage divider compensation)
     */
    int32_t get_voltage_mv() const;

    /**
     * @brief Get latest voltage reading in volts (thread-safe)
     * @return Voltage in V (after voltage divider compensation)
     */
    float get_voltage_v() const;

    /**
     * @brief Check if valid readings are available
     * @return true if at least one reading has been taken
     */
    bool has_valid_reading() const;

private:
    VoltageSensor() = default;
    ~VoltageSensor();

    static void task_entry(void* arg);
    void sampling_task();

    Config config_;
    adc_oneshot_unit_handle_t adc_handle_{nullptr};
    adc_cali_handle_t cali_handle_{nullptr};
    bool calibrated_{false};
    float voltage_divider_ratio_{1.0f};

    TaskHandle_t task_handle_{nullptr};
    std::atomic<int32_t> latest_voltage_mv_{0};
    std::atomic<bool> valid_reading_{false};

    static constexpr const char* TAG = "VoltageSensor";
};
