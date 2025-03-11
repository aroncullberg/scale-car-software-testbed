#pragma once

#include <atomic>

#include "SBUS.h"
// #include "gps.h"
#include "imu.h"
#include "sensor_types.h"

class VehicleData {
    public:
    // [SIGNLETON] Access method (global?)
    static VehicleData& instance() {
        static VehicleData instance; // Should be thread safe (?)
        return instance;
    }

    // thread safe (i think since only one task will call it?) update methods
    void updateSBUS(const sensor::SbusData& data);
    void updateGPS(const sensor::GpsData& data);
    void updateIMU(const sensor::ImuData& data);

    // Unsure if this is threadsafe, should be fine since its read and not write(?)
    sensor::SbusData getSbus() const;
    sensor::GpsData getGPS() const;
    sensor::ImuData getImu() const;

    uint32_t getSbusTimestamp() const {return sbus_timestamp_.load() ; }
    uint32_t getGPSTimestamp() const {return gps_timestamp_.load() ; }
    uint32_t getImuTimestamp() const {return imu_timestamp_.load() ; }
    
    
    private:
    // [SINGLETON] private constructor to prevent direct instantiation
    VehicleData() = default;

    // [SINGLETON] Delete compy constructor and assignment operator
    VehicleData(const VehicleData&) = delete;
    VehicleData& operator =(const VehicleData&) = delete;

    sensor::SbusData sbus_{};
    sensor::GpsData gps_{};
    sensor::ImuData imu_{};

    std::atomic<uint32_t> sbus_timestamp_{0};
    std::atomic<uint32_t> gps_timestamp_{0};
    std::atomic<uint32_t> imu_timestamp_{0};
};