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

#include <map>
#include <string>
#include <vector>

class LogMonitor {
public:
    struct Config {
        const char* ap_ssid{"ESP32-Monitor"};
        const char* ap_password{"password"};
        uint16_t config_port{8888};
        uint16_t log_port{7777};

        uint32_t server_task_stack_size{8192};
        uint8_t server_task_priority{5};

        size_t log_queue_size{50};
    };

    static LogMonitor& instance();

    esp_err_t init(const Config& config);
    esp_err_t start();
    esp_err_t stop();

    void queueLogMessage(const char* message);
    void processCommand(const char* command_line, int client_socket);

private:
    LogMonitor() = default;
    ~LogMonitor();

    // Delete copy constructor and assignment operator
    LogMonitor(const LogMonitor&) = delete;
    LogMonitor& operator=(const LogMonitor&) = delete;

    void handleGetCommand(const char* module, const char* setting, int client_socket);
    void handleSetCommand(const char* module, const char* setting, const char* value, int client_socket);
    void handleHelpCommand(int client_socket);
    void handleListCommand(const char* module, int client_socket);

    void sendResponse(const char* response, int client_socket);

    static void serverTask(void* args);
    esp_err_t setupWiFi();
    esp_err_t initQueue();

    Config config_;
    QueueHandle_t log_queue_ = nullptr;
    TaskHandle_t server_task_ = nullptr;
    bool is_running_ = false;
    int server_socket_ = -1;

    int config_server_socket_ = -1;
    int log_server_socket_ = -1;

    // std::vector<int> client_sockets_;
    std::map<int, bool> client_sockets_; // client_fd -> is_config_client

};

// Global hook function
int log_monitor_vprintf(const char* format, va_list args);
#endif //LOG_MONITOR_H
