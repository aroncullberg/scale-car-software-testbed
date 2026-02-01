#include "voltage_sensor.h"
#include "esp_log.h"
#include <algorithm>

VoltageSensor& VoltageSensor::instance() {
    static VoltageSensor instance;
    return instance;
}

VoltageSensor::~VoltageSensor() {
    stop();

    if (cali_handle_) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_delete_scheme_curve_fitting(cali_handle_);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_delete_scheme_line_fitting(cali_handle_);
#endif
    }

    if (adc_handle_) {
        adc_oneshot_del_unit(adc_handle_);
    }
}

esp_err_t VoltageSensor::init(const Config& config) {
    config_ = config;

    // Calculate voltage divider ratio
    voltage_divider_ratio_ = 1.0f * static_cast<float>(config_.resistor_r1_ohms + config_.resistor_r2_ohms) /
                             static_cast<float>(config_.resistor_r2_ohms);

    // Configure ADC unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,  // Most ESP32 variants support ADC1
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure ADC channel
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = config_.attenuation,
        .bitwidth = ADC_BITWIDTH_12,  // 12-bit resolution (0-4095)
    };

    ret = adc_oneshot_config_channel(adc_handle_, config_.adc_channel, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize ADC calibration
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        // .chan = config_.adc_channel,
        .atten = config_.attenuation,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle_);
    if (ret == ESP_OK) {
        calibrated_ = true;
        ESP_LOGI(TAG, "ADC calibration: curve fitting");
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = config_.attenuation,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle_);
    if (ret == ESP_OK) {
        calibrated_ = true;
        ESP_LOGI(TAG, "ADC calibration: line fitting");
    }
#endif

    if (!calibrated_) {
        ESP_LOGW(TAG, "ADC calibration not available, using raw values");
    }

    ESP_LOGI(TAG, "Voltage sensor initialized - R1=%lu Ω, R2=%lu Ω, ratio=%.2f",
             config_.resistor_r1_ohms, config_.resistor_r2_ohms, voltage_divider_ratio_);

    if (config_.enable_telemetry) {
        ESP_LOGI(TAG, "Battery telemetry enabled - interval=%lums, capacity=%lumAh",
                 config_.telemetry_interval_ms, config_.battery_capacity_mah);
    }

    return ESP_OK;
}

esp_err_t VoltageSensor::start() {
    if (task_handle_ != nullptr) {
        ESP_LOGW(TAG, "Sampling task already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        task_entry,
        "voltage_sensor",
        3072,               // Stack size (increased for telemetry)
        this,               // Pass this pointer
        3,                  // Priority (lower than control tasks)
        &task_handle_,
        0                   // Pin to core 0
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sampling task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Voltage sensor sampling task started");
    return ESP_OK;
}

void VoltageSensor::stop() {
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
        ESP_LOGI(TAG, "Voltage sensor sampling task stopped");
    }
}

void VoltageSensor::task_entry(void* arg) {
    auto* sensor = static_cast<VoltageSensor*>(arg);
    sensor->sampling_task();
}

void VoltageSensor::sampling_task() {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(config_.sample_interval_ms);

    while (true) {
        int32_t total_mv = 0;
        uint32_t valid_samples = 0;

        // Take multiple samples and average
        for (uint32_t i = 0; i < config_.samples_per_read; i++) {
            int adc_raw = 0;
            esp_err_t ret = adc_oneshot_read(adc_handle_, config_.adc_channel, &adc_raw);
            // ESP_LOGI(TAG, "ADC raw: %d", adc_raw);

            if (ret == ESP_OK) {
                int voltage_mv = 0;

                if (calibrated_) {
                    // Convert raw value to millivolts using calibration
                    ret = adc_cali_raw_to_voltage(cali_handle_, adc_raw, &voltage_mv);
                    if (ret == ESP_OK) {
                        total_mv += voltage_mv;
                        valid_samples++;
                    }
                } else {
                    // Fallback: linear approximation (less accurate)
                    // For ADC_ATTEN_DB_11: 0-4095 maps to 0-3300mV approximately
                    voltage_mv = (adc_raw * 3300) / 4095;
                    total_mv += voltage_mv;
                    valid_samples++;
                }
            }

            // Small delay between samples
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (valid_samples > 0) {
            // Calculate average ADC voltage
            int32_t avg_adc_mv = total_mv / valid_samples;

            // Apply voltage divider compensation
            int32_t actual_voltage_mv = static_cast<int32_t>(avg_adc_mv * voltage_divider_ratio_);

            // Store atomically
            latest_voltage_mv_.store(actual_voltage_mv, std::memory_order_relaxed);
            valid_reading_.store(true, std::memory_order_relaxed);
        }

        vTaskDelayUntil(&last_wake_time, interval);
    }
}

int32_t VoltageSensor::get_voltage_mv() const {
    return latest_voltage_mv_.load(std::memory_order_relaxed);
}

float VoltageSensor::get_voltage_v() const {
    return latest_voltage_mv_.load(std::memory_order_relaxed) / 1000.0f;
}

bool VoltageSensor::has_valid_reading() const {
    return valid_reading_.load(std::memory_order_relaxed);
}
