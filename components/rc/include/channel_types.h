//
// Created by aron on 2025-04-26.
//

#pragma once
#include <cstdint>

namespace rclink
{
/**
* @brief Enum class representing the channel indices (1..16)
*/
enum class ChannelIndex: std::uint8_t
{
    CH1 = 0, CH2, CH3, CH4, CH5, CH6, CH7, CH8,
    CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16,
    COUNT
};

inline constexpr std::size_t k_channel_count = static_cast<std::size_t>(ChannelIndex::COUNT);

/**
* @brief Scaled channel value that is expected to be within the range 0 - 2000.
*/
using channel_value_t = uint16_t;

/**
* @brief Raw and scaled channel data pair
*/
struct ChannelPair {
    uint16_t raw;           // Raw protocol value (varies by protocol)
    channel_value_t scaled; // Scaled 0-2000 value
};

/**
* @brief All RC channels with .ch1.raw/.ch1.scaled access pattern
*/
struct RcChannels {
    ChannelPair ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8,
                ch9, ch10, ch11, ch12, ch13, ch14, ch15, ch16;
};

// struct Channel
// {
// private:
//     channel_value_t value_;
//
// public:
//     constexpr explicit Channel(channel_value_t value) noexcept : value_(value)
//     {
//     }
//
//     constexpr bool low() const noexcept { return value_ < static_cast<channel_value_t>(25); }
//
//     constexpr bool mid() const noexcept
//     {
//         return value_ >= static_cast<channel_value_t>(1475) && value_ <= static_cast<channel_value_t>(1525);
//     }
//
//     constexpr bool high() const noexcept { return value_ > static_cast<channel_value_t>(1975); }
//
//     constexpr channel_value_t raw() const noexcept { return value_; }
//     constexpr operator channel_value_t() const noexcept { return value_; }
// };

/**
* @brief Telemetry data structures for RC communication
*/
struct BatteryTelemetry {
    uint16_t voltage_mv;    // Battery voltage in millivolts
    uint16_t current_ma;    // Current draw in milliamps
    uint32_t capacity_mah;  // Battery capacity in mAh
    uint8_t remaining_pct;  // Remaining battery percentage (0-100)
};

struct GpsTelemetry {
    int32_t latitude_1e7;   // Latitude * 10,000,000 (degrees)
    int32_t longitude_1e7;  // Longitude * 10,000,000 (degrees)
    uint16_t speed_kmh;     // Ground speed in km/h * 10
    uint16_t heading_deg;   // Heading in degrees * 100
    uint16_t altitude_m;    // Altitude in meters + 1000m offset
    uint8_t satellites;     // Number of satellites
};

struct AttitudeTelemetry {
    float roll_deg;         // Roll angle in degrees
    float pitch_deg;        // Pitch angle in degrees
    float yaw_deg;          // Yaw angle in degrees
};

struct LinkStatsTelemetry {
    uint8_t rssi;           // RSSI value
    uint8_t link_quality;   // Link quality percentage
    uint8_t snr;            // Signal-to-noise ratio
    uint16_t good_frames;   // Number of good frames received
    uint16_t bad_frames;    // Number of bad frames received
};

struct AirspeedTelemetry {
    uint16_t speed_kmh_x10; // Speed in km/h * 10 (0.1 km/h resolution)
};

struct FlightModeTelemetry {
    char mode[32];          // Flight/drive mode string (null-terminated)
};

struct TempTelemetry {
    uint8_t source_id;      // Temperature source (0=FC/ESCs, 1=Ambient, etc.)
    int16_t temps[8];       // Temperature values in deci-degrees Celsius (e.g., 250 = 25.0°C)
    uint8_t count;          // Number of valid temperature values (max 20, but we use 8 for RC car)
};

struct RpmTelemetry {
    uint8_t source_id;      // RPM source (0=Motor group 1, etc.)
    int32_t rpm_values[4];  // RPM values (negative for reverse rotation)
    uint8_t count;          // Number of valid RPM values (max 19, but we use 4 for RC car motors)
};
}
