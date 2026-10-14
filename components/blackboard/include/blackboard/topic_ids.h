#pragma once
#include <cstdint>

namespace blackboard {

// Central, dependency-free registry of topic ids. Append only, never renumber.
// The owning component binds its type to one of these via a Topic<T>
// specialisation (see blackboard.h). The offline log decoder reads this list.
enum class TopicId : uint8_t {
    reserved = 0,
    battery = 1,
    gps = 2,
    euler = 3,
    link_stats = 4,
    rc_channels = 5,
    rpm = 6,
    flight_mode = 7,
    accel = 8,
    gyro = 9,
    quat = 10,
};

}
