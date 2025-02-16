#pragma once

// #include <cstdint>

// Version control for data structures
struct DataVersion {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
};

// Common timestamp structure
struct Timestamp {
    uint32_t millis;      // Milliseconds since boot
    uint32_t micros;      // Microseconds part
};

// Quality metrics base structure
struct QualityMetrics {
    bool valid_data;
    uint32_t error_count;
    float update_rate_hz;
};