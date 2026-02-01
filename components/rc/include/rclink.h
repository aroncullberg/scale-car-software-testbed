//
// Created by aron on 2025-04-26.
//

#pragma once

#include <array>
#include <atomic>

#include "esp_err.h"

#include "channel_types.h"
#include "rc_backend.h"


namespace proto
{
class SbusDriver;
};

namespace rclink
{
class Backend;

/**
 * @brief Configuration for RC receiver auto-detection
 */
struct ReceiverConfig {
    uint32_t per_protocol_timeout_ms = 1000;  // Time to test each protocol before moving to next
};

/**
 * @brief Singleton facade that gives application code read-only access to the latest RC channel values,
 * should make it easier to have it be independent of the underlying protocol
 */
class Receiver
{
public:
    static Receiver& instance();

    bool valid_data() const noexcept { return valid_; }

    // Primary interface - get all channels at once
    RcChannels get_channels() const;


    // Auto-detection setup method
    esp_err_t setup(const uart_pins_t& pins, const ReceiverConfig& config = ReceiverConfig{});

    esp_err_t register_backend(const Backend* b);

    // Telemetry transmission methods
    esp_err_t send_battery(const BatteryTelemetry& data);
    esp_err_t send_gps(const GpsTelemetry& data);
    esp_err_t send_attitude(const AttitudeTelemetry& data);
    esp_err_t send_link_stats(const LinkStatsTelemetry& data);
    esp_err_t send_airspeed(const AirspeedTelemetry& data);
    esp_err_t send_flight_mode(const FlightModeTelemetry& data);
    esp_err_t send_temp(const TempTelemetry& data);
    esp_err_t send_rpm(const RpmTelemetry& data);
    bool supports_telemetry() const;

    Receiver(const Receiver&) = delete; // Disable copy constructor
    Receiver& operator=(const Receiver&) = delete; // Disable copy assignment operator

private:
    friend class Backend;
    Receiver() = default;

    void push_from_backend(const Backend* caller, ChannelIndex channel, uint16_t raw_value, channel_value_t scaled_value);
    void set_valid_data(const bool v) { valid_ = v;}

    const Backend* backend_{nullptr};
    bool valid_{false};

    std::atomic<RcChannels> channels_{};
};
}
