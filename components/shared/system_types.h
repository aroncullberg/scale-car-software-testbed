#pragma once

#include <stdint.h>  // Replace cstdint with stdint.h

struct TaskStats {
    char task_name[16];
    uint32_t stack_high_water_mark;
    uint8_t priority;
    uint8_t current_state;
    uint32_t runtime_counter;
    float cpu_usage_percent;
};

struct SystemStats {
    // Memory stats
    uint32_t free_heap_size;
    uint32_t min_free_heap;
    uint8_t heap_fragmentation;
    
    // CPU stats
    float cpu_frequency_mhz;
    float cpu_temperature_c;
    uint8_t cpu_usage_percent;
    
    // Task stats
    uint8_t task_count;
    TaskStats tasks[16];  // Adjust size based on needs
    
    // WiFi/Network stats
    int8_t wifi_rssi;
    uint32_t wifi_tx_packets;
    uint32_t wifi_rx_packets;
    
    // Timing stats
    uint32_t uptime_seconds;
    float loop_time_us;
};

struct CompactSystemStats {
    // Memory stats
    uint32_t free_heap_size;
    uint32_t min_free_heap;
    uint8_t heap_fragmentation;
    
    // CPU stats
    float cpu_frequency_mhz;
    float cpu_temperature_c;
    uint8_t cpu_usage_percent;
    
    // Task summary (instead of full task details)
    uint8_t task_count;
    uint32_t max_task_watermark;  // Highest watermark across all tasks
    
    // WiFi/Network stats
    int8_t wifi_rssi;
    
    // Timing stats
    uint32_t uptime_seconds;
    float loop_time_us;
};