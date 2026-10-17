#include "recording/ulog.hpp"

#include <array>
#include <cstring>

namespace recording {
namespace {
struct Bytes {
    std::array<uint8_t, 256> data{};
    size_t size = 0;
    void u8(uint8_t v) { data[size++] = v; }
    void u16(uint16_t v) { u8(v); u8(v >> 8); }
    void u32(uint32_t v) { u16(v); u16(v >> 16); }
    void u64(uint64_t v) { u32(v); u32(v >> 32); }
    void f32(float v) { uint32_t bits; std::memcpy(&bits, &v, 4); u32(bits); }
    void text(const char* text) { const auto n = std::strlen(text); std::memcpy(data.data() + size, text, n); size += n; }
};
constexpr const char* formats[] = {
    "car_control:uint64_t timestamp;uint32_t tuning_revision;float throttle;float brake;float steering_input;float steering_center;float steering_range;float front_aero;float rear_aero;float yaw_target_rads;float yaw_feedback_rads;float gyro_z_rads;float steering_output;float p_term;float i_term;float d_term;uint32_t gyro_age_us;uint16_t[4] motor_commands;uint8_t rc_usable;uint8_t armed;uint8_t arm_requested;uint8_t direction;uint8_t steering_mode;uint8_t yaw_active;uint8_t gyro_valid;",
    "car_power:uint64_t timestamp;int32_t voltage_mv;int32_t current_ma;uint8_t valid;",
    "car_tuning:uint64_t timestamp;uint32_t revision;float max_yaw_deg_s;float p;float i;float d;float d_cutoff_hz;float integral_limit;",
    "car_accel:uint64_t timestamp;float x_ms2;float y_ms2;float z_ms2;uint8_t accuracy;uint8_t valid;",
    "car_gyro:uint64_t timestamp;float x_rads;float y_rads;float z_rads;uint8_t accuracy;uint8_t valid;",
    "car_quat:uint64_t timestamp;float w;float x;float y;float z;float accuracy_rad;uint8_t accuracy;uint8_t valid;",
    "car_euler:uint64_t timestamp;float roll_deg;float pitch_deg;float yaw_deg;float accuracy_rad;uint8_t accuracy;uint8_t valid;",
    "car_gps:uint64_t timestamp;int32_t lat_e7;int32_t lon_e7;int32_t altitude_mm;int32_t speed_mmps;int32_t course_cd;uint32_t satellites;uint32_t location_age_ms;uint16_t hdop_centi;uint16_t year;uint8_t month;uint8_t day;uint8_t hour;uint8_t minute;uint8_t second;uint8_t centisecond;uint16_t updated_fields;uint16_t valid_fields;",
    "car_rpm:uint64_t timestamp;uint32_t rpm;uint32_t erpm;uint32_t period_us;uint16_t motor_poles;",
};
constexpr const char* names[] = {"car_control", "car_power", "car_tuning", "car_accel",
    "car_gyro", "car_quat", "car_euler", "car_gps", "car_rpm"};
constexpr uint16_t rpm_id = 8;
}

bool ULog::message(char type, const void* data, size_t size) {
    std::array<uint8_t, 1024> message{};
    if (size > message.size() - 3) return false;
    message[0] = uint8_t(size); message[1] = uint8_t(size >> 8); message[2] = uint8_t(type);
    std::memcpy(message.data() + 3, data, size);
    return sink_(context_, message.data(), size + 3);
}

bool ULog::begin(uint64_t timestamp_us) {
    Bytes header;
    for (auto byte : {0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35, 0x01}) header.u8(byte);
    header.u64(timestamp_us);
    if (!sink_(context_, header.data.data(), header.size)) return false;
    const std::array<uint8_t, 40> flags{};
    if (!message('B', flags.data(), flags.size())) return false;
    for (const auto format : formats) if (!message('F', format, std::strlen(format))) return false;
    Bytes info;
    constexpr const char* key = "char[3] sys_name";
    info.u8(std::strlen(key)); info.text(key); info.text("car");
    if (!message('I', info.data.data(), info.size)) return false;
    info = {};
    constexpr const char* version_key = "char[6] ver_sw";
    info.u8(std::strlen(version_key)); info.text(version_key); info.text("2026.1");
    if (!message('I', info.data.data(), info.size)) return false;
    for (uint16_t id = 0; id < std::size(names); ++id) {
        const size_t instances = id == rpm_id ? vehicle::motor_count : 1;
        for (size_t motor = 0; motor < instances; ++motor) {
            Bytes subscription;
            subscription.u8(motor); subscription.u16(id + motor); subscription.text(names[id]);
            if (!message('A', subscription.data.data(), subscription.size)) return false;
        }
    }
    Bytes motors;
    constexpr const char* motor_key = "char[11] motor_order";
    motors.u8(std::strlen(motor_key)); motors.text(motor_key); motors.text("FL,RR,RL,FR");
    if (!message('I', motors.data.data(), motors.size)) return false;
    return true;
}

bool ULog::control(const vehicle::ControlSample& s) {
    Bytes data;
    data.u16(0); data.u64(s.timestamp_us); data.u32(s.tuning_revision);
    for (const auto value : {s.command.throttle, s.command.brake, s.command.steering,
            s.command.steering_center_adjustment, s.command.steering_range, s.command.front_aero,
            s.command.rear_aero, s.steering.requested_rads, s.steering.measured_rads,
            s.gyro_z_rads, s.steering.output, s.steering.proportional, s.steering.integral,
            s.steering.derivative}) data.f32(value);
    data.u32(s.gyro_age_us);
    for (const auto value : s.motor_commands) data.u16(value);
    data.u8(s.rc_usable); data.u8(s.armed); data.u8(s.command.arm);
    data.u8(uint8_t(s.command.direction)); data.u8(uint8_t(s.command.steering_mode));
    data.u8(s.steering.active); data.u8(s.gyro_valid);
    return message('D', data.data.data(), data.size);
}

bool ULog::power(uint64_t timestamp_us, const PowerMonitor::Reading& reading) {
    Bytes data;
    data.u16(1); data.u64(timestamp_us); data.u32(reading.voltage_mv);
    data.u32(reading.current_ma); data.u8(reading.valid);
    return message('D', data.data.data(), data.size);
}

bool ULog::accel(const imu::AccelData& s) {
    Bytes data;
    data.u16(3); data.u64(s.timestamp_us);
    data.f32(s.x_ms2); data.f32(s.y_ms2); data.f32(s.z_ms2);
    data.u8(uint8_t(s.accuracy)); data.u8(s.valid);
    return message('D', data.data.data(), data.size);
}

bool ULog::gyro(const imu::GyroData& s) {
    Bytes data;
    data.u16(4); data.u64(s.timestamp_us);
    data.f32(s.x_rads); data.f32(s.y_rads); data.f32(s.z_rads);
    data.u8(uint8_t(s.accuracy)); data.u8(s.valid);
    return message('D', data.data.data(), data.size);
}

bool ULog::quat(const imu::QuatData& s) {
    Bytes data;
    data.u16(5); data.u64(s.timestamp_us);
    for (const auto value : {s.w, s.x, s.y, s.z, s.accuracy_rad}) data.f32(value);
    data.u8(uint8_t(s.accuracy)); data.u8(s.valid);
    return message('D', data.data.data(), data.size);
}

bool ULog::euler(const imu::EulerData& s) {
    Bytes data;
    data.u16(6); data.u64(s.timestamp_us);
    for (const auto value : {s.roll_deg, s.pitch_deg, s.yaw_deg, s.accuracy_rad}) data.f32(value);
    data.u8(uint8_t(s.accuracy)); data.u8(s.valid);
    return message('D', data.data.data(), data.size);
}

bool ULog::gps(const nav::GpsSample& s) {
    Bytes data;
    data.u16(7); data.u64(s.timestamp_us);
    data.u32(s.lat_e7); data.u32(s.lon_e7); data.u32(s.altitude_mm);
    data.u32(s.speed_mmps); data.u32(s.course_cd); data.u32(s.satellites); data.u32(s.location_age_ms);
    data.u16(s.hdop_centi); data.u16(s.year);
    data.u8(s.month); data.u8(s.day); data.u8(s.hour); data.u8(s.minute); data.u8(s.second); data.u8(s.centisecond);
    data.u16(s.updated_fields); data.u16(s.valid_fields);
    return message('D', data.data.data(), data.size);
}

bool ULog::rpm(size_t motor, const dshot::ErpmSample& s) {
    if (motor >= vehicle::motor_count || s.updated_at_us <= 0) return false;
    Bytes data;
    data.u16(rpm_id + motor); data.u64(s.updated_at_us);
    data.u32(s.rpm); data.u32(s.erpm); data.u32(s.period_us); data.u16(s.motor_poles);
    return message('D', data.data.data(), data.size);
}

bool ULog::parameter(const char* name, float value) {
    Bytes data;
    data.u8(6 + std::strlen(name)); data.text("float "); data.text(name); data.f32(value);
    return message('P', data.data.data(), data.size);
}

bool ULog::tuning(uint64_t timestamp_us, uint32_t revision, const vehicle::YawRateConfig& c) {
    const float max_deg = c.maximum_yaw_rate_rads * 57.295779513f;
    Bytes data;
    data.u16(2); data.u64(timestamp_us); data.u32(revision);
    for (const auto value : {max_deg, c.proportional_gain, c.integral_gain, c.derivative_gain,
                             c.derivative_cutoff_hz, c.integral_limit}) data.f32(value);
    if (!message('D', data.data.data(), data.size)) return false;
    return parameter("YAW_RATE_MAX_DEG_S", max_deg) && parameter("YAW_RATE_P", c.proportional_gain) &&
        parameter("YAW_RATE_I", c.integral_gain) && parameter("YAW_RATE_D", c.derivative_gain) &&
        parameter("YAW_D_CUTOFF_HZ", c.derivative_cutoff_hz) && parameter("YAW_I_LIMIT", c.integral_limit);
}

bool ULog::dropout(uint16_t duration_ms) {
    Bytes data; data.u16(duration_ms);
    return message('O', data.data.data(), data.size);
}
}
