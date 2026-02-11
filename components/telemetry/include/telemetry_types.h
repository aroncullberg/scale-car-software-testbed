#pragma once
#include <cstdint>

namespace telemetry {

struct BatteryTelemetry {
    uint16_t voltage_mv;
    uint16_t current_ma;
    uint32_t capacity_mah;
    uint8_t remaining_pct;
};

struct GpsTelemetry {
    int32_t latitude_1e7;
    int32_t longitude_1e7;
    uint16_t speed_kmh;
    uint16_t heading_deg;
    uint16_t altitude_m;
    uint8_t satellites;
};

struct AttitudeTelemetry {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
};

struct LinkStatsTelemetry {
    uint8_t rssi;
    uint8_t link_quality;
    uint8_t snr;
    uint16_t good_frames;
    uint16_t bad_frames;
};

struct AirspeedTelemetry {
    uint16_t speed_kmh_x10;
};

struct FlightModeTelemetry {
    char mode[32];
};

struct TempTelemetry {
    uint8_t source_id;
    int16_t temps[8];
    uint8_t count;
};

struct RpmTelemetry {
    uint8_t source_id;
    int32_t rpm_values[4];
    uint8_t count;
};

struct AccelGyroTelemetry {
    uint32_t sample_time_us;
    float gyro_x_rads;
    float gyro_y_rads;
    float gyro_z_rads;
    float accel_x_ms2;
    float accel_y_ms2;
    float accel_z_ms2;
};

}
