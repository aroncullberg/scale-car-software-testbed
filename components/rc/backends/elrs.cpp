//
// Created by cullb on 2025-04-26.
//

#include "elrs.h"
#include "rgb_led.h"

extern "C" {
    #include "ESP_CRSF.h"
}

using rclink::channel_value_t;
using rclink::ChannelIndex;

#undef TAG
#define TAG "ExpressLRS"

namespace proto
{
ExpressLRS::ExpressLRS(const Config& cfg) : cfg_(cfg)
{
};

ExpressLRS::~ExpressLRS()
{
    ExpressLRS::stop();
    ESP_LOGI(TAG, "ExpressLRS instance destroyed");
}

esp_err_t ExpressLRS::start(const rclink::uart_pins_t& pins, uint32_t timeout_ms)
{
    ESP_LOGD(TAG, "Starting ELRS on UART%d (RX: %d, TX: %d)", pins.uart_num, pins.rx_pin, pins.tx_pin);

    if (running_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Update config with provided pins
    cfg_.uart_num = pins.uart_num;
    cfg_.uart_rx_pin = pins.rx_pin;
    cfg_.uart_tx_pin = pins.tx_pin;

    // Don't suppress CRSF logging during detection - we need debug visibility
    esp_log_level_set("uart", ESP_LOG_WARN);
    // esp_log_level_set("CRSF", ESP_LOG_WARN);  // Keep CRSF logs visible for debugging

    // Initialize CRSF with our pins
    crsf_config_t config = {
        .uart_num = static_cast<uint8_t>(cfg_.uart_num),
        .tx_pin = static_cast<uint8_t>(cfg_.uart_tx_pin),
        .rx_pin = static_cast<uint8_t>(cfg_.uart_rx_pin)
    };

    esp_err_t ret = CRSF_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CRSF_init failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    // Try to detect valid CRSF frames within timeout
    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    crsf_channels_t channels = {};
    uint32_t valid_frames = 0;
    const uint32_t required_frames = 2;

    ESP_LOGD(TAG, "Detecting ELRS/CRSF frames for %lu ms...", timeout_ms);

    while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
        if (CRSF_receive_channels(&channels) == ESP_OK) {
            if (channels.ch1 > 0 && channels.ch1 < 2048) {
                valid_frames++;
                ESP_LOGI(TAG, "✓ Valid CRSF frame %lu (ch1: %d, ch2: %d, ch3: %d, ch4: %d)",
                         valid_frames, channels.ch1, channels.ch2, channels.ch3, channels.ch4);

                if (valid_frames >= required_frames) {
                    ESP_LOGI(TAG, "✓ ELRS protocol detected! Starting main task...");

                    // Start the ExpressLRS task for continuous operation
                    BaseType_t res = xTaskCreatePinnedToCore(
                        taskEntry,
                        "ExpressLRS task",
                        4096,
                        this,
                        5,
                        &task_,
                        1
                    );

                    running_ = (res == pdPASS);
                    if (running_) {
                        ESP_LOGI(TAG, "✓ ExpressLRS backend task started successfully");
                    } else {
                        ESP_LOGE(TAG, "✗ Failed to start ExpressLRS backend task");
                    }
                    return running_ ? ESP_OK : ESP_FAIL;
                }
            } else {
                ESP_LOGD(TAG, "Frame rejected: ch1=%d out of range", channels.ch1);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGD(TAG, "ELRS detection timeout (%lu valid frames, needed %lu)", valid_frames, required_frames);
    CRSF_cleanup();
    return ESP_ERR_TIMEOUT;
}

esp_err_t ExpressLRS::stop()
{
    if (!running_)
        return ESP_ERR_INVALID_STATE;
    vTaskDelete(task_);
    task_ = nullptr;
    running_ = false;
    return ESP_OK;
}



void ExpressLRS::taskEntry(void* arg)
{
    auto* self = static_cast<ExpressLRS*>(arg);
    self->run();
}

void ExpressLRS::run()
{
    crsf_channels_t channels = {};
    TickType_t last_wake_time = xTaskGetTickCount();
    bool was_valid = false;
    bool was_armed = false;
    uint32_t valid_frames = 0;
    uint32_t invalid_frames = 0;

    // Arming threshold: CH2 >= 1500 (11-bit center ~992) means armed
    constexpr uint16_t ARM_THRESHOLD = 992;  // Center point in 11-bit CRSF

    ESP_LOGI(TAG, "ExpressLRS run loop started (CH2 arming threshold: %d)", ARM_THRESHOLD);

    while (true) {
        esp_err_t result = CRSF_receive_channels(&channels);

        if (result == ESP_OK) {
            // Link is active and data is fresh
            valid_frames++;

            // Check arming status from CH2 (ELRS standard)
            bool armed = (channels.ch5 >= ARM_THRESHOLD);

            // Handle link state transition
            if (!was_valid) {
                ESP_LOGI(TAG, "✓ RC link established (valid=%lu, invalid=%lu)", valid_frames, invalid_frames);
                was_valid = true;
            }

            // Handle arming state transition
            if (armed != was_armed) {
                if (armed) {
                    // ESP_LOGI(TAG, "✓ ARMED (CH2: %d >= %d)", channels.ch2, ARM_THRESHOLD);
                    led::Rgb::instance().set_color(0, 255, 0);  // Green = armed
                } else {
                    // ESP_LOGI(TAG, "✓ DISARMED (CH2: %d < %d)", channels.ch2, ARM_THRESHOLD);
                    led::Rgb::instance().set_color(0, 0, 255);  // Blue = disarmed
                }
                was_armed = armed;
            }

            set_valid_data(true);

            // Push all 16 channels
            push(ChannelIndex::CH1, channels.ch1, rawToScaled(channels.ch1));
            push(ChannelIndex::CH2, channels.ch2, rawToScaled(channels.ch2));
            push(ChannelIndex::CH3, channels.ch3, rawToScaled(channels.ch3));
            push(ChannelIndex::CH4, channels.ch4, rawToScaled(channels.ch4));
            push(ChannelIndex::CH5, channels.ch5, rawToScaled(channels.ch5));
            push(ChannelIndex::CH6, channels.ch6, rawToScaled(channels.ch6));
            push(ChannelIndex::CH7, channels.ch7, rawToScaled(channels.ch7));
            push(ChannelIndex::CH8, channels.ch8, rawToScaled(channels.ch8));
            push(ChannelIndex::CH9, channels.ch9, rawToScaled(channels.ch9));
            push(ChannelIndex::CH10, channels.ch10, rawToScaled(channels.ch10));
            push(ChannelIndex::CH11, channels.ch11, rawToScaled(channels.ch11));
            push(ChannelIndex::CH12, channels.ch12, rawToScaled(channels.ch12));
            push(ChannelIndex::CH13, channels.ch13, rawToScaled(channels.ch13));
            push(ChannelIndex::CH14, channels.ch14, rawToScaled(channels.ch14));
            push(ChannelIndex::CH15, channels.ch15, rawToScaled(channels.ch15));
            push(ChannelIndex::CH16, channels.ch16, rawToScaled(channels.ch16));

            // Log channel values periodically (every 100 valid frames)
            if (valid_frames % 100 == 0) {
                ESP_LOGD(TAG, "Channels: ch1=%d ch2=%d ch3=%d ch4=%d (raw, armed=%s)",
                         channels.ch1, channels.ch2, channels.ch3, channels.ch4,
                         armed ? "yes" : "no");
            }
        }
        else if (result == ESP_ERR_TIMEOUT) {
            // Link lost - data is stale
            invalid_frames++;

            if (was_valid) {
                ESP_LOGW(TAG, "✗ RC link lost (valid=%lu, invalid=%lu)", valid_frames, invalid_frames);
                led::Rgb::instance().set_color(255, 0, 0);  // Red = no link
                was_valid = false;
                was_armed = false;  // Reset arming state on link loss
            }

            set_valid_data(false);
        }
        else {
            // Other error (shouldn't happen in normal operation)
            ESP_LOGE(TAG, "CRSF_receive_channels error: %s", esp_err_to_name(result));
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

esp_err_t ExpressLRS::send_battery(const rclink::BatteryTelemetry& data) const
{
    if (!running_) {
        ESP_LOGW(TAG, "Cannot send battery telemetry - ELRS not running");
        return ESP_ERR_INVALID_STATE;
    }

    // Convert to CRSF format
    crsf_battery_t crsf_battery = {
        .voltage = static_cast<uint16_t>(data.voltage_mv / 100),  // Convert mv to 0.1V units
        .current = static_cast<uint16_t>(data.current_ma / 100),  // Convert ma to 0.1A units
        .capacity = data.capacity_mah,     // mAh value (24-bit)
        .remaining = data.remaining_pct    // Percentage (0-100)
    };

    CRSF_send_battery_data(CRSF_DEST_FC, &crsf_battery);
    return ESP_OK;
}

esp_err_t ExpressLRS::send_gps(const rclink::GpsTelemetry& data) const
{
    if (!running_) {
        ESP_LOGW(TAG, "Cannot send GPS telemetry - ELRS not running");
        return ESP_ERR_INVALID_STATE;
    }

    // Convert to CRSF format
    crsf_gps_t crsf_gps = {
        .latitude = data.latitude_1e7,     // Already in correct format
        .longitude = data.longitude_1e7,   // Already in correct format
        .groundspeed = data.speed_kmh,     // Already in km/h * 10
        .heading = data.heading_deg,       // Already in degrees * 100
        .altitude = data.altitude_m,       // Already with +1000m offset
        .satellites = data.satellites      // Satellite count
    };

    CRSF_send_gps_data(CRSF_DEST_FC, &crsf_gps);
    return ESP_OK;
}

esp_err_t ExpressLRS::send_attitude(const rclink::AttitudeTelemetry& data) const
{
    if (!running_) {
        ESP_LOGW(TAG, "Cannot send attitude telemetry - ELRS not running");
        return ESP_ERR_INVALID_STATE;
    }

    // Convert float degrees to int16 with LSB = 100 µrad
    // 1 radian = 1,000,000 µrad
    // 1 degree = (π / 180) radians = 17,453.29 µrad
    // So degrees * 174.5329 = value in units of 100 µrad
    constexpr float deg_to_crsf = 174.5329f;

    crsf_attitude_t crsf_attitude = {
        .pitch = static_cast<int16_t>(data.pitch_deg * deg_to_crsf),
        .roll = static_cast<int16_t>(data.roll_deg * deg_to_crsf),
        .yaw = static_cast<int16_t>(data.yaw_deg * deg_to_crsf)
    };

    CRSF_send_attitude_data(CRSF_DEST_FC, &crsf_attitude);
    return ESP_OK;
}

esp_err_t ExpressLRS::send_airspeed(const rclink::AirspeedTelemetry& data) const
{
    if (!running_) {
        ESP_LOGW(TAG, "Cannot send airspeed telemetry - ELRS not running");
        return ESP_ERR_INVALID_STATE;
    }

    // Convert to CRSF format - data.speed_kmh_x10 is already in 0.1 km/h units
    crsf_airspeed_t crsf_airspeed = {
        .speed = data.speed_kmh_x10
    };

    CRSF_send_airspeed_data(CRSF_DEST_FC, &crsf_airspeed);
    return ESP_OK;
}

esp_err_t ExpressLRS::send_flight_mode(const rclink::FlightModeTelemetry& data) const
{
    if (!running_) {
        ESP_LOGW(TAG, "Cannot send flight mode telemetry - ELRS not running");
        return ESP_ERR_INVALID_STATE;
    }

    // Flight mode is just a null-terminated string
    CRSF_send_flight_mode_data(CRSF_DEST_FC, data.mode);
    return ESP_OK;
}

esp_err_t ExpressLRS::send_temp(const rclink::TempTelemetry& data) const
{
    if (!running_) {
        ESP_LOGW(TAG, "Cannot send temperature telemetry - ELRS not running");
        return ESP_ERR_INVALID_STATE;
    }

    // Temperature data is already in correct format (deci-degrees Celsius)
    // Just pass it directly to CRSF
    CRSF_send_temp_data(CRSF_DEST_FC, data.source_id, data.temps, data.count);
    return ESP_OK;
}

esp_err_t ExpressLRS::send_rpm(const rclink::RpmTelemetry& data) const
{
    if (!running_) {
        ESP_LOGW(TAG, "Cannot send RPM telemetry - ELRS not running");
        return ESP_ERR_INVALID_STATE;
    }

    // RPM data is passed as int32_t and will be packed as 24-bit by CRSF layer
    CRSF_send_rpm_data(CRSF_DEST_FC, data.source_id, data.rpm_values, data.count);
    return ESP_OK;
}

}
