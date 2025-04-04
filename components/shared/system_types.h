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

enum class Frequency : uint16_t {
    F1000Hz = 1,   // 1000 / 1 = 1000 Hz
    F500Hz  = 2,   // 1000 / 2 = 500 Hz
    F333Hz  = 3,   // 1000 / 3 ≈ 333 Hz
    F250Hz  = 4,   // 1000 / 4 = 250 Hz
    F200Hz  = 5,   // 1000 / 5 = 200 Hz
    F166Hz  = 6,   // 1000 / 6 ≈ 166 Hz
    F142Hz  = 7,   // 1000 / 7 ≈ 142 Hz
    F125Hz  = 8,   // 1000 / 8 = 125 Hz
    F111Hz  = 9,   // 1000 / 9 ≈ 111 Hz
    F100Hz  = 10,  // 1000 / 10 = 100 Hz
    F83Hz   = 12,  // 1000 / 12 ≈ 83 Hz
    F71Hz   = 14,  // 1000 / 14 ≈ 71 Hz
    F62Hz   = 16,  // 1000 / 16 = 62 Hz
    F50Hz   = 20,  // 1000 / 20 = 50 Hz
    F40Hz   = 25,  // 1000 / 25 = 40 Hz
    F25Hz   = 40,  // 1000 / 40 = 25 Hz
    F20Hz   = 50,  // 1000 / 50 = 20 Hz
    F10Hz   = 100  // 1000 / 100 = 10 Hz
};



