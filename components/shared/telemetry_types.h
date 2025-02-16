#pragma once

// #include <cstdint>


enum class PacketType : uint8_t {
    SENSOR_DATA = 0x01,
    SYSTEM_STATS = 0x02,
    DEBUG_MSG = 0x03,
    COMMAND = 0x04
};

struct PacketHeader {
    uint8_t magic;            // Magic byte for validation
    PacketType type;          // Packet type
    uint8_t sequence;         // Sequence number
    uint16_t length;          // Payload length
    Timestamp timestamp;      // Packet timestamp
};