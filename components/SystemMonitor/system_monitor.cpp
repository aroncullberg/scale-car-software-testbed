#include "system_monitor.h"
#include "esp_system.h"
#include "esp_cpu.h"
#include "esp_chip_info.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <cstring>
// #include "esp_cpu.h"

static auto TAG = "SysMonitor";

esp_err_t SystemMonitor::init(const Config& config) {
    if (is_running_) {
        return ESP_ERR_INVALID_STATE;
    }
    
    config_ = config;
    return ESP_OK;
}

esp_err_t SystemMonitor::start() {
    return ESP_OK; // WARNING: REMOVE
    if (is_running_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Create the monitoring task
    BaseType_t ret = xTaskCreate(
        monitorTask,
        "sys_monitor",
        config_.stack_size,
        this,
        config_.task_priority,
        &task_handle_
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        return ESP_FAIL;
    }

    is_running_ = true;
    return ESP_OK;
}

esp_err_t SystemMonitor::stop() {
    if (!is_running_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    is_running_ = false;
    return ESP_OK;
}

SystemStats SystemMonitor::getStats() const {
    // Return a copy of the current stats
    return current_stats_;
}

void SystemMonitor::monitorTask(void* params) {
    auto* monitor = static_cast<SystemMonitor*>(params);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        // Record start time for loop timing
        int64_t start_time = esp_timer_get_time();

        // Update all stats
        monitor->updateStats();

        // Calculate and store loop time
        monitor->current_stats_.loop_time_us = 
            static_cast<float>(esp_timer_get_time() - start_time);

        // Wait for next period
        vTaskDelayUntil(&last_wake_time, monitor->config_.update_period);
    }
}

void SystemMonitor::updateStats() {
    // Memory stats
    current_stats_.free_heap_size = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    current_stats_.min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    
    // Heap fragmentation calculation
    // size_t free_blocks = heap_caps_get_num_free_blocks(MALLOC_CAP_DEFAULT);
    // size_t total_blocks = heap_caps_get_num_free_blocks(MALLOC_CAP_DEFAULT) + 
    //                      heap_caps_get_allocated_size(MALLOC_CAP_DEFAULT);
                            // Replace the fragmentation calculation with this:
    const size_t total_free = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    const size_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    current_stats_.heap_fragmentation = static_cast<uint8_t>(100 - (largest_free_block * 100) / total_free);
    // current_stats_.heap_fragmentation = static_cast<uint8_t>((free_blocks * 100) / total_blocks);

    // CPU stats
    // TODO: Fix or remove this
    // current_stats_.cpu_frequency_mhz = static_cast<float>(esp_cpu_get_freq_hz()) / 1000000.0f;
    // Update temperature (if available on your ESP32 variant)
    #ifdef CONFIG_ESP_SYSTEM_TEMP_SENSOR_ENABLE
        temp_sensor_config_t temp_config = TEMP_SENSOR_CONFIG_DEFAULT();
        temp_sensor_get_config(&temp_config);
        float tsens_value;
        temp_sensor_read_celsius(&tsens_value);
        current_stats_.cpu_temperature_c = tsens_value;
    #else
        current_stats_.cpu_temperature_c = 0.0f;
    #endif

    // Task stats
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    current_stats_.task_count = static_cast<uint8_t>(
        task_count > 16 ? 16 : task_count); // Limit to array size

    // Get task stats
    TaskStatus_t task_details[16];
    task_count = uxTaskGetSystemState(
        task_details,
        current_stats_.task_count,
        nullptr  // Total runtime not needed
    );

    // Update task stats
    for (size_t i = 0; i < task_count && i < 16; i++) {
        TaskStats& stats = current_stats_.tasks[i];
        
        // Copy task name (safely)
        strlcpy(stats.task_name, task_details[i].pcTaskName, 
                sizeof(stats.task_name));
        
        stats.stack_high_water_mark = task_details[i].usStackHighWaterMark;
        stats.priority = task_details[i].uxCurrentPriority;
        stats.current_state = task_details[i].eCurrentState;
        stats.runtime_counter = task_details[i].ulRunTimeCounter;
        
        // Calculate CPU usage (simplified)
        stats.cpu_usage_percent = 0.0f;  // Proper CPU % calculation needs runtime deltas
    }

    // WiFi stats (if WiFi is initialized)
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        current_stats_.wifi_rssi = ap_info.rssi;
    } else {
        current_stats_.wifi_rssi = 0;
    }

    // Update uptime
    current_stats_.uptime_seconds = esp_timer_get_time() / 1000000;
}