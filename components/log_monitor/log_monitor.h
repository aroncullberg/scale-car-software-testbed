//
// Created by cullb on 2025-02-26.
//

#ifndef LOG_MONITOR_H
#define LOG_MONITOR_H

#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <string>
#include <vector>

class LogMonitor {
public:
    struct Config {
        const char* ap_ssid{"ESP32-Monitor"};
        const char* ap_password{"password"};
        uint16_t tcp_port{8888};

        uint32_t server_task_stack_size{4096};
        uint8_t server_task_priority{5};

        size_t log_queue_size{50};
    };

    static LogMonitor& instance();

    esp_err_t init(const Config& config);
    esp_err_t start();
    esp_err_t stop();

    void queueLogMessage(const char* message);

private:
    LogMonitor() = default;
    ~LogMonitor();

    // Delete copy constructor and assignment operator
    LogMonitor(const LogMonitor&) = delete;
    LogMonitor& operator=(const LogMonitor&) = delete;

    static void serverTask(void* args);
    esp_err_t setupWiFi();
    esp_err_t initQueue();

    Config config_;
    QueueHandle_t log_queue_ = nullptr;
    TaskHandle_t server_task_ = nullptr;
    bool is_running_ = false;
    int server_socket_ = -1;

    std::vector<int> client_sockets_;
};

// Global hook function
int log_monitor_vprintf(const char* format, va_list args);
#endif //LOG_MONITOR_H
