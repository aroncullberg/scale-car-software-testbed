#include "power_monitor.h"
#include "telemetry.h"
#include "esp_log.h"
#include <algorithm>

struct LutEntry {
    uint16_t cell_mv;
    uint8_t pct;
};

// LiPo per-cell voltage to remaining capacity (light-load approximation)
static constexpr LutEntry lipo_lut[] = {
    {4200, 100}, {4150,  95}, {4110,  90}, {4080,  85},
    {4020,  80}, {3980,  75}, {3920,  70}, {3870,  65},
    {3830,  60}, {3790,  55}, {3750,  50}, {3710,  45},
    {3680,  40}, {3650,  35}, {3620,  30}, {3580,  25},
    {3530,  20}, {3490,  15}, {3450,  10}, {3400,   5},
    {3300,   0},
};
static constexpr size_t lipo_lut_size = sizeof(lipo_lut) / sizeof(lipo_lut[0]);

PowerMonitor& PowerMonitor::instance() {
    static PowerMonitor inst;
    return inst;
}

PowerMonitor::~PowerMonitor() {
    stop();

    auto delete_cali = [](adc_cali_handle_t h) {
        if (!h) return;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_delete_scheme_curve_fitting(h);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_delete_scheme_line_fitting(h);
#endif
    };

    delete_cali(voltage_cali_handle_);
    delete_cali(current_cali_handle_);

    if (voltage_adc_handle_) adc_oneshot_del_unit(voltage_adc_handle_);
    if (current_adc_handle_) adc_oneshot_del_unit(current_adc_handle_);
}

static esp_err_t init_adc(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                           adc_oneshot_unit_handle_t* out_handle,
                           adc_cali_handle_t* out_cali, bool* out_calibrated) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_cfg, out_handle);
    if (ret != ESP_OK) return ret;

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_oneshot_config_channel(*out_handle, channel, &chan_cfg);
    if (ret != ESP_OK) return ret;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, out_cali);
    if (ret == ESP_OK) *out_calibrated = true;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_cfg, out_cali);
    if (ret == ESP_OK) *out_calibrated = true;
#endif

    return ESP_OK;
}

esp_err_t PowerMonitor::init(const Config& config) {
    config_ = config;

    voltage_divider_ratio_ = config_.voltage_divider_ratio;

    esp_err_t ret = init_adc(config_.voltage_adc_unit, config_.voltage_channel, config_.attenuation,
                              &voltage_adc_handle_, &voltage_cali_handle_, &voltage_calibrated_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Voltage ADC init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = init_adc(config_.current_adc_unit, config_.current_channel, config_.attenuation,
                    &current_adc_handle_, &current_cali_handle_, &current_calibrated_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Current ADC init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Initialized — V: ratio=%.2f | I: scale=%u (x0.1 mV/A) | %u cells %lumAh",
             voltage_divider_ratio_, config_.current_scale_x10mv_per_a,
             config_.cell_count, config_.battery_capacity_mah);

    return ESP_OK;
}

esp_err_t PowerMonitor::start() {
    if (task_handle_) {
        ESP_LOGW(TAG, "Already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t r = xTaskCreatePinnedToCore(
        task_entry, "power_mon", 3072, this, 3, &task_handle_, 0);

    if (r != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Sampling task started");
    return ESP_OK;
}

void PowerMonitor::stop() {
    if (task_handle_) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
        ESP_LOGI(TAG, "Sampling task stopped");
    }
}

void PowerMonitor::task_entry(void* arg) {
    static_cast<PowerMonitor*>(arg)->sampling_task();
}

int32_t PowerMonitor::read_adc_mv(adc_oneshot_unit_handle_t handle, adc_channel_t channel,
                                   adc_cali_handle_t cali, bool calibrated) {
    int32_t total_mv = 0;
    uint32_t valid = 0;

    for (uint32_t i = 0; i < config_.samples_per_read; i++) {
        int raw = 0;
        if (adc_oneshot_read(handle, channel, &raw) != ESP_OK) continue;

        int mv = 0;
        if (calibrated && adc_cali_raw_to_voltage(cali, raw, &mv) == ESP_OK) {
            total_mv += mv;
            valid++;
        } else if (!calibrated) {
            mv = (raw * 3300) / 4095;
            total_mv += mv;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return valid > 0 ? total_mv / static_cast<int32_t>(valid) : 0;
}

int32_t PowerMonitor::current_from_adc_mv(int32_t adc_mv) const {
    if (config_.current_scale_x10mv_per_a == 0) return 0;
    int32_t mv = adc_mv - current_offset_mv_;
    if (mv < 0) mv = 0;
    return mv * 10000 / config_.current_scale_x10mv_per_a;
}

uint8_t PowerMonitor::voltage_to_remaining_pct(int32_t voltage_mv) const {
    if (config_.cell_count == 0) return 0;
    uint16_t cell_mv = static_cast<uint16_t>(voltage_mv / config_.cell_count);

    if (cell_mv >= lipo_lut[0].cell_mv) return 100;
    if (cell_mv <= lipo_lut[lipo_lut_size - 1].cell_mv) return 0;

    for (size_t i = 0; i < lipo_lut_size - 1; i++) {
        if (cell_mv >= lipo_lut[i + 1].cell_mv) {
            uint16_t v_hi = lipo_lut[i].cell_mv;
            uint16_t v_lo = lipo_lut[i + 1].cell_mv;
            uint8_t p_hi = lipo_lut[i].pct;
            uint8_t p_lo = lipo_lut[i + 1].pct;
            return static_cast<uint8_t>(p_lo + (cell_mv - v_lo) * (p_hi - p_lo) / (v_hi - v_lo));
        }
    }
    return 0;
}

void PowerMonitor::sampling_task() {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(config_.sample_interval_ms);

    while (true) {
        int32_t v_adc_mv = read_adc_mv(voltage_adc_handle_, config_.voltage_channel,
                                         voltage_cali_handle_, voltage_calibrated_);
        int32_t v_mv = static_cast<int32_t>(v_adc_mv * voltage_divider_ratio_);

        int32_t i_adc_mv = read_adc_mv(current_adc_handle_, config_.current_channel,
                                         current_cali_handle_, current_calibrated_);

        if (zero_cal_count_ < ZERO_CAL_SAMPLES) {
            if (i_adc_mv < zero_cal_min_) zero_cal_min_ = i_adc_mv;
            zero_cal_count_++;
            if (zero_cal_count_ == ZERO_CAL_SAMPLES) {
                current_offset_mv_ = zero_cal_min_;
                ESP_LOGI(TAG, "Current sensor auto-zero: offset=%ldmV", current_offset_mv_);
            }
            vTaskDelayUntil(&last_wake, interval);
            continue;
        }

        int32_t i_ma = current_from_adc_mv(i_adc_mv);

        consumed_mah_accum_ += static_cast<double>(i_ma) * config_.sample_interval_ms / 3600000.0;
        uint32_t consumed = static_cast<uint32_t>(consumed_mah_accum_);
        uint8_t pct = voltage_to_remaining_pct(v_mv);

        voltage_mv_.store(v_mv, std::memory_order_relaxed);
        current_ma_.store(i_ma, std::memory_order_relaxed);
        consumed_mah_.store(consumed, std::memory_order_relaxed);
        remaining_pct_.store(pct, std::memory_order_relaxed);
        valid_reading_.store(true, std::memory_order_relaxed);

        telemetry::BatteryTelemetry bat{};
        bat.voltage_mv = static_cast<uint16_t>(std::min(v_mv, (int32_t)UINT16_MAX));
        bat.current_ma = static_cast<uint16_t>(std::min(i_ma, (int32_t)UINT16_MAX));
        bat.capacity_mah = consumed;
        bat.remaining_pct = pct;
        telemetry::publish(bat);

        // ESP_LOGI(TAG, "V:%ldmV  I:%ldmA (adc:%ldmV)  used:%lumAh  rem:%u%%",
        //          v_mv, i_ma, i_adc_mv, consumed, pct);

        vTaskDelayUntil(&last_wake, interval);
    }
}

int32_t PowerMonitor::get_voltage_mv() const {
    return voltage_mv_.load(std::memory_order_relaxed);
}

int32_t PowerMonitor::get_current_ma() const {
    return current_ma_.load(std::memory_order_relaxed);
}

uint32_t PowerMonitor::get_consumed_mah() const {
    return consumed_mah_.load(std::memory_order_relaxed);
}

uint8_t PowerMonitor::get_remaining_pct() const {
    return remaining_pct_.load(std::memory_order_relaxed);
}

bool PowerMonitor::has_valid_reading() const {
    return valid_reading_.load(std::memory_order_relaxed);
}
