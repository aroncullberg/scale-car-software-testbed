//
// Created by cullb on 2025-04-26.
//

#include <esp_log.h>
#include <rclink.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "channel_types.h"
#include "backends/elrs.h"
#include "backends/sbus.h"
#include "rgb_led.h"

using namespace rclink;

auto tag = "rc-interface";

Receiver& Receiver::instance()
{
    static Receiver instance;
    return instance;
}

RcChannels Receiver::get_channels() const
{
    return channels_.load(std::memory_order_relaxed);
}

esp_err_t Receiver::register_backend(const Backend* b)
{
    if (!b) {
        ESP_LOGE(tag, "register_backend called with nullptr");
        return ESP_ERR_INVALID_ARG;
    }
    if (backend_) {
        ESP_LOGE(tag, "cannot register backend, one is already set");
        return ESP_ERR_INVALID_STATE;
    }
    backend_ = b;
    ESP_LOGI(tag, "registered backend");
    return ESP_OK;
}

void Receiver::push_from_backend(const Backend* caller, ChannelIndex channel, uint16_t raw_value, channel_value_t scaled_value)
{
    if (caller != backend_) {
        ESP_LOGE("Receiver", "Unregistered backend tried to push data");
        return;
    }

    auto channels = channels_.load(std::memory_order_relaxed);
    ChannelPair* channel_array = &channels.ch1;
    channel_array[static_cast<size_t>(channel)].raw = raw_value;
    channel_array[static_cast<size_t>(channel)].scaled = scaled_value;
    channels_.store(channels, std::memory_order_relaxed);
}

// Backend method implementations
void Backend::push(ChannelIndex channel, uint16_t raw_value, channel_value_t scaled_value) const
{
    Receiver::instance().push_from_backend(this, channel, raw_value, scaled_value);
}

void Backend::set_valid_data(bool valid) const
{
    bool previous = Receiver::instance().valid_data();
    Receiver::instance().set_valid_data(valid);

    if (previous != valid) {
        if (valid) {
            led::Rgb::instance().set_color(0, 0, 255);
            ESP_LOGI(tag, "RC data valid");
        } else {
            led::Rgb::instance().set_color(255, 0, 0);
            ESP_LOGW(tag, "RC data invalid");
        }
    }
}

esp_err_t Receiver::setup(const uart_pins_t& pins, const ReceiverConfig& config)
{
    if (backend_) {
        ESP_LOGE(tag, "setup() called but backend already registered");
        return ESP_ERR_INVALID_STATE;
    }

    led::Rgb::instance().set_color(255, 0, 0);

    ESP_LOGI(tag, "Starting RC auto-detection on UART%d (RX: %d, TX: %d)",
             pins.uart_num, pins.rx_pin, pins.tx_pin);
    ESP_LOGD(tag, "Config: per_protocol_timeout=%lums", config.per_protocol_timeout_ms);

    // Create protocol backends as static - they need to persist if successful
    static proto::ExpressLRS elrs_backend({});
    static proto::SbusDriver sbus_backend({});

    // Test protocols in order: ELRS -> SBUS
    Backend* backends[] = {&elrs_backend, &sbus_backend};
    const char* protocol_names[] = {"ELRS", "SBUS"};

    // Retry forever until a protocol is detected
    while (true) {
        for (size_t i = 0; i < sizeof(backends) / sizeof(backends[0]); ++i) {
            ESP_LOGD(tag, "Trying %s protocol...", protocol_names[i]);

            backend_ = backends[i];

            esp_err_t result = backends[i]->start(pins, config.per_protocol_timeout_ms);
            if (result == ESP_OK) {
                ESP_LOGI(tag, "RC setup complete with %s protocol", protocol_names[i]);
                return ESP_OK;
            }

            ESP_LOGW(tag, "Failed to detect %s", protocol_names[i]);

            backend_ = nullptr;
        }
    }
}

esp_err_t Receiver::send_battery(const telemetry::BatteryTelemetry &data) {
    if (!backend_) {
        ESP_LOGE(tag, "No backend registered for telemetry transmission");
        return ESP_ERR_INVALID_STATE;
    }
    return backend_->send_battery(data);
}

esp_err_t Receiver::send_gps(const telemetry::GpsTelemetry &data) {
    if (!backend_) {
        ESP_LOGE(tag, "No backend registered for telemetry transmission");
        return ESP_ERR_INVALID_STATE;
    }
    return backend_->send_gps(data);
}

esp_err_t Receiver::send_attitude(const telemetry::AttitudeTelemetry& data)
{
    if (!backend_) {
        ESP_LOGE(tag, "No backend registered for telemetry transmission");
        return ESP_ERR_INVALID_STATE;
    }
    return backend_->send_attitude(data);
}

esp_err_t Receiver::send_link_stats(const telemetry::LinkStatsTelemetry& data)
{
    if (!backend_) {
        ESP_LOGE(tag, "No backend registered for telemetry transmission");
        return ESP_ERR_INVALID_STATE;
    }
    return backend_->send_link_stats(data);
}

esp_err_t Receiver::send_airspeed(const telemetry::AirspeedTelemetry& data)
{
    if (!backend_) {
        ESP_LOGE(tag, "No backend registered for telemetry transmission");
        return ESP_ERR_INVALID_STATE;
    }
    return backend_->send_airspeed(data);
}

esp_err_t Receiver::send_flight_mode(const telemetry::FlightModeTelemetry& data)
{
    if (!backend_) {
        ESP_LOGE(tag, "No backend registered for telemetry transmission");
        return ESP_ERR_INVALID_STATE;
    }
    return backend_->send_flight_mode(data);
}

esp_err_t Receiver::send_temp(const telemetry::TempTelemetry& data)
{
    if (!backend_) {
        ESP_LOGE(tag, "No backend registered for telemetry transmission");
        return ESP_ERR_INVALID_STATE;
    }
    return backend_->send_temp(data);
}

esp_err_t Receiver::send_rpm(const telemetry::RpmTelemetry& data)
{
    if (!backend_) {
        ESP_LOGE(tag, "No backend registered for telemetry transmission");
        return ESP_ERR_INVALID_STATE;
    }
    return backend_->send_rpm(data);
}

esp_err_t Receiver::send_accelgyro(const telemetry::AccelGyroTelemetry& data)
{
    if (!backend_) {
        ESP_LOGE(tag, "No backend registered for telemetry transmission");
        return ESP_ERR_INVALID_STATE;
    }
    return backend_->send_accelgyro(data);
}

bool Receiver::supports_telemetry() const
{
    if (!backend_) {
        return false;
    }
    return backend_->supports_telemetry();
}
