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
    class SbusDriver: public rclink::Backend
    {
    public:
        struct Config {
            uart_port_t uart_num{UART_NUM_1};
            gpio_num_t uart_tx_pin{GPIO_NUM_NC};
            gpio_num_t uart_rx_pin{GPIO_NUM_NC};
            int buad_rate{100000};
            TickType_t tick_period{pdMS_TO_TICKS(14)}; // ~70Hz
        };

        // SBUS protocol UART configuration
        static constexpr uart_config_t SBUS_UART_CONFIG = {
            .baud_rate = 100000,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_EVEN,
            .stop_bits = UART_STOP_BITS_2,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_APB,
            .flags = {}
        };

        explicit SbusDriver(const Config& cfg);
        ~SbusDriver();

        // Simplified interface
        esp_err_t start(const rclink::uart_pins_t& pins, uint32_t timeout_ms) override;
        esp_err_t stop() override;
        bool supports_telemetry() const override { return false; }

        SbusDriver(const SbusDriver&) = delete;
        SbusDriver& operator=(const SbusDriver&) = delete;

    private:
        /* === Internal helpers === */
        esp_err_t configureUART();
        static void taskEntry(void *arg);
        void run();
        void processFrame(const uint8_t *frame, size_t length);

        /* === Lookup table to scale raw 11-bit values to 0-2000 === */
        static constexpr uint16_t RAW_MIN = 192;
        static constexpr uint16_t RAW_MAX = 1792;
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

        /* === constants === */
        static constexpr size_t FRAME_SIZE = 25;
        static constexpr uint8_t START_BYTE = 0x0F;
        static constexpr uint8_t END_BYTE = 0x00;
        static constexpr size_t WINDOW = 128;

        /* === members === */
        Config cfg_;
        TaskHandle_t task_{nullptr};
        bool running_{false};
        QueueHandle_t uart_evt_q_{nullptr};

        std::atomic<uint32_t> total_frames_{0};
        std::atomic<uint32_t> lost_frames_{0};
    };
}



