#pragma once

#include <cstdint>

namespace Telemetry {
    // Forward declarations(?) for sensor data
    namespace sensor {
        struct ImuData;
        struct GPSData;
        struct SbusData;
    }

    // Shared telemetry queue item definition
    struct TelemetryQueueItem {
        enum class PacketType : uint8_t {
            COMMAND = 0x01,
            TEXT = 0x02,
            SENSOR = 0x03,
            HEARTHBEAT = 0x04,
        };

        struct TextMessageData {
            uint32_t timestamp;
            uint8_t severity;
            char text[120];
        };

        PacketType type;
        uint32_t timestamp;
        union PacketData {
            sensor::ImuData* imu;   // Use pointers or forward-declared types
            sensor::GPSData* gps;
            sensor::SbusData* sbus;
            TextMessageData text;
            PacketData() : imu(nullptr) {} // Initialize union safely
        } data;
    };
}