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

protected:
    void push(ChannelIndex channel, uint16_t raw_value, channel_value_t scaled_value) const;
    void set_valid_data(bool valid) const;
};
}
