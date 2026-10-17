#pragma once

#include <cstddef>
#include <cstdint>
#include "vehicle/control_sample.hpp"
#include "power_monitor.h"
#include "imu_types.h"
#include "gps_types.h"
#include "dshot/telemetry.hpp"

namespace recording {
class ULog {
public:
    using Sink = bool (*)(void*, const void*, size_t);
    ULog(Sink sink, void* context) : sink_(sink), context_(context) {}
    bool begin(uint64_t timestamp_us);
    bool control(const vehicle::ControlSample& sample);
    bool power(uint64_t timestamp_us, const PowerMonitor::Reading& reading);
    bool accel(const imu::AccelData& sample);
    bool gyro(const imu::GyroData& sample);
    bool quat(const imu::QuatData& sample);
    bool euler(const imu::EulerData& sample);
    bool gps(const nav::GpsSample& sample);
    bool rpm(size_t motor, const dshot::ErpmSample& sample);
    bool tuning(uint64_t timestamp_us, uint32_t revision, const vehicle::YawRateConfig& config);
    bool dropout(uint16_t duration_ms);
private:
    bool message(char type, const void* data, size_t size);
    bool parameter(const char* name, float value);
    Sink sink_;
    void* context_;
};
}
