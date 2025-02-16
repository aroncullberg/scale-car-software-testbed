// system_monitor.h
#pragma once

#include "system_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include <stdint.h>  // Replace cstdint with stdint.h

class SystemMonitor {
public:
    struct Config {
        TickType_t update_period{pdMS_TO_TICKS(1000)}; // Default 1 second updates
        uint8_t task_priority{2};  // Lower priority since it's just monitoring
        uint32_t stack_size{4096}; // Typically doesn't need much stack
    };

    static SystemMonitor& instance() {
        static SystemMonitor instance;
        return instance;
    }

    esp_err_t init(const Config& config);
    esp_err_t start();
    esp_err_t stop();

    // Get the latest stats (thread-safe)
    SystemStats getStats() const;

private:
    SystemMonitor() = default;
    ~SystemMonitor() = default;
    
    // Delete copy/move
    SystemMonitor(const SystemMonitor&) = delete;
    SystemMonitor& operator=(const SystemMonitor&) = delete;

    static void monitorTask(void* params);
    void updateStats();

    Config config_{};
    TaskHandle_t task_handle_{nullptr};
    SystemStats current_stats_{};
    bool is_running_{false};
};