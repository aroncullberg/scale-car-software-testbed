#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

class PowerMonitor {
public:
    struct Reading {
        int32_t voltage_mv;
        int32_t current_ma;
        uint32_t consumed_mah;
        uint8_t remaining_pct;
        uint8_t cell_count;
        bool valid;
    };

    struct Config {
        // Voltage channel (battery via voltage divider)
        adc_unit_t voltage_adc_unit;
        adc_channel_t voltage_channel;
        float voltage_divider_ratio;  // Vbattery / Vadc (measure both, divide)

        // Current channel (ESC analog output)
        adc_unit_t current_adc_unit;
        adc_channel_t current_channel;
        uint16_t current_scale_x10mv_per_a; // Betaflight convention: e.g. 90 = 9 mV/A

        // Shared ADC settings
        adc_atten_t attenuation = ADC_ATTEN_DB_12;
        uint32_t samples_per_read = 16;
        uint32_t sample_interval_ms = 100;

        // Battery config
        uint32_t battery_capacity_mah;
        uint8_t cell_count;
    };

    static PowerMonitor& instance();

    PowerMonitor(const PowerMonitor&) = delete;
    PowerMonitor& operator=(const PowerMonitor&) = delete;

    esp_err_t init(const Config& config);
    using ReadingObserver = void (*)(void*, uint64_t, const Reading&);
    esp_err_t set_observer(ReadingObserver observer, void* context);
    esp_err_t start();
    void stop();
    Reading reading() const;

private:
    PowerMonitor() = default;
    ~PowerMonitor();

    static void task_entry(void* arg);
    void sampling_task();

    int32_t read_adc_mv(adc_oneshot_unit_handle_t handle, adc_channel_t channel,
                        adc_cali_handle_t cali, bool calibrated);
    int32_t current_from_adc_mv(int32_t adc_mv) const;
    uint8_t voltage_to_remaining_pct(int32_t voltage_mv) const;

    Config config_{};
    float voltage_divider_ratio_{1.0f};

    adc_oneshot_unit_handle_t voltage_adc_handle_{nullptr};
    adc_cali_handle_t voltage_cali_handle_{nullptr};
    bool voltage_calibrated_{false};

    adc_oneshot_unit_handle_t current_adc_handle_{nullptr};
    adc_cali_handle_t current_cali_handle_{nullptr};
    bool current_calibrated_{false};

    TaskHandle_t task_handle_{nullptr};

    // Coulomb counting accumulator (not atomic — only written from task)
    double consumed_mah_accum_{0.0};

    // Auto-zero calibration for current sensor
    static constexpr uint32_t ZERO_CAL_SAMPLES = 10;
    uint32_t zero_cal_count_{0};
    int32_t zero_cal_min_{INT32_MAX};
    int32_t current_offset_mv_{0};

    mutable portMUX_TYPE reading_lock_ = portMUX_INITIALIZER_UNLOCKED;
    Reading reading_{};
    ReadingObserver observer_ = nullptr;
    void* observer_context_ = nullptr;
};
