//
// Created by aron on 06/03/2025.
//

#pragma once

#include <cstdint>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "channel_types.h"

namespace rclink
{

struct uart_pins_t {
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    uart_port_t uart_num;
};

// Forward declaration
class Receiver;

class Backend
{
public:
    virtual ~Backend() = default;

    // Simplified lifecycle methods
    virtual esp_err_t start(const uart_pins_t& pins, uint32_t timeout_ms) = 0;  // Setup + detect + start
    virtual esp_err_t stop() = 0;
    virtual bool supports_telemetry() const = 0;

    // Telemetry transmission methods
    virtual esp_err_t send_battery(const BatteryTelemetry& data) const { return ESP_ERR_NOT_SUPPORTED; }
    virtual esp_err_t send_gps(const GpsTelemetry& data) const { return ESP_ERR_NOT_SUPPORTED; }
    virtual esp_err_t send_attitude(const AttitudeTelemetry& data) const { return ESP_ERR_NOT_SUPPORTED; }
    virtual esp_err_t send_link_stats(const LinkStatsTelemetry& data) const { return ESP_ERR_NOT_SUPPORTED; }
    virtual esp_err_t send_airspeed(const AirspeedTelemetry& data) const { return ESP_ERR_NOT_SUPPORTED; }
    virtual esp_err_t send_flight_mode(const FlightModeTelemetry& data) const { return ESP_ERR_NOT_SUPPORTED; }
    virtual esp_err_t send_temp(const TempTelemetry& data) const { return ESP_ERR_NOT_SUPPORTED; }
    virtual esp_err_t send_rpm(const RpmTelemetry& data) const { return ESP_ERR_NOT_SUPPORTED; }

protected:
    void push(ChannelIndex channel, uint16_t raw_value, channel_value_t scaled_value) const;
    void set_valid_data(bool valid) const;
};
}
