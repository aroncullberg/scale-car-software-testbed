//
// Created by aron on 2025-04-27.
//

#pragma once

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "gps.h"
#include "TinyGPS++.h"

namespace proto
{

    class NmeaDriver {
    public:
        struct Config {
            uart_port_t uart_num{UART_NUM_1};
            gpio_num_t uart_tx_pin{GPIO_NUM_NC};
            gpio_num_t uart_rx_pin{GPIO_NUM_NC};
            int buad_rate{38400};
            size_t rx_buffer_size{2048};
            size_t tx_buffer_size{0};
            int pattern_queue_size{16};
            uint32_t task_stack_size{4096};
            uint8_t task_priority {5};
            TickType_t tick_period{pdMS_TO_TICKS(20)};
        };

        explicit NmeaDriver(Config &cfg);
        ~NmeaDriver();

        esp_err_t init();
        esp_err_t start();
        esp_err_t stop();

        NmeaDriver(const NmeaDriver &) = delete;
        NmeaDriver &operator=(const NmeaDriver &) = delete;

    private:
        esp_err_t configureUart();
        static void taskEntry(void *arg);
        void run();
        void ingest(const char* data, size_t len);

        static constexpr char TAG[]         = "NmeaDriver";
        static constexpr char PATTERN_CHAR  = '\n';
        static constexpr size_t QUEUE_LEN   = 16;
        static constexpr size_t SENTENCE_MAX = 512;

        Config          cfg_{};
        TaskHandle_t    task_{nullptr};
        QueueHandle_t   uart_queue_{nullptr};
        bool            running_{false};
        TinyGPSPlus     gps_{};
    };

}