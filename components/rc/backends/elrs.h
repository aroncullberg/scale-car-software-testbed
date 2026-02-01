//
// Created by cullb on 2025-04-26.
//

#pragma once

#include <cstdint>
#include <array>
#include <algorithm>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "rclink.h"
#include "rc_backend.h"

namespace rclink {
class Backend;
}

namespace proto
{
    class ExpressLRS : public rclink::Backend
    {
    public:
        struct Config {
            uart_port_t uart_num{UART_NUM_1};
            gpio_num_t uart_tx_pin{GPIO_NUM_NC};
            gpio_num_t uart_rx_pin{GPIO_NUM_NC};
        };

        // ELRS/CRSF protocol UART configuration
        static constexpr uart_config_t ELRS_UART_CONFIG = {
            .baud_rate = 420000,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_APB,
            .flags = {}
        };

        explicit ExpressLRS(const Config& cfg);
        ~ExpressLRS();

        // Simplified interface
        esp_err_t start(const rclink::uart_pins_t& pins, uint32_t timeout_ms) override;
        esp_err_t stop() override;
        bool supports_telemetry() const override { return true; }

        // Telemetry transmission methods
        esp_err_t send_battery(const rclink::BatteryTelemetry& data) const override;
        esp_err_t send_gps(const rclink::GpsTelemetry& data) const override;
        esp_err_t send_attitude(const rclink::AttitudeTelemetry& data) const override;
        esp_err_t send_airspeed(const rclink::AirspeedTelemetry& data) const override;
        esp_err_t send_flight_mode(const rclink::FlightModeTelemetry& data) const override;
        esp_err_t send_temp(const rclink::TempTelemetry& data) const override;
        esp_err_t send_rpm(const rclink::RpmTelemetry& data) const override;

        ExpressLRS(const ExpressLRS&) = delete;
        ExpressLRS& operator=(const ExpressLRS&) = delete;

    private:
        static void taskEntry(void *arg);
        void run();

        static constexpr uint16_t RAW_MIN = 172;
        static constexpr uint16_t RAW_MAX = 1810;
        static constexpr uint16_t RAW_RANGE = RAW_MAX - RAW_MIN;

        inline static constexpr auto SCALE_LUT = []{
            std::array<uint16_t, RAW_RANGE + 1> lut{};
            for (uint32_t i = 0; i <= RAW_RANGE; ++i) {
                lut[i] = static_cast<uint16_t>((i * 2000u + RAW_RANGE / 2) / RAW_RANGE);
            }
            return lut;
        }();

        static constexpr rclink::channel_value_t rawToScaled(const uint16_t raw) {
            const uint16_t clamped = std::clamp(raw, RAW_MIN, RAW_MAX);
            return SCALE_LUT[clamped - RAW_MIN];
        }

        /* === members === */
        Config cfg_;
        TaskHandle_t task_{nullptr};
        bool running_{false};
    };
}



