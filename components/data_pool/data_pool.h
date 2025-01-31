#pragma once

#include <atomic>

#include "SBUS.h"
#include "gps.h"

class VehicleData {
    public:
    // [SIGNLETON] Access method (global?)
    static VehicleData& instance() {
        static VehicleData instance; // Should be thread safe (?)
        return instance;
    }

    // thread safe (i think since only one task will call it?) update methods
    void updateSBUS(const sensor::SbusData& data);
    void updateGPS(const sensor::GPSData& data);

    // Unsure if this is threadsafe, should be fine since its read and not write(?)
    sensor::SbusData getSbus() const;
    sensor::GPSData getGPS() const;

    uint32_t getSbusTimestamp() const {return sbus_timestamp_.load() ; }
    uint32_t getGPSTimestamp() const {return gps_timestamp_.load() ; }
    
    
    private:
    // [SINGLETON] private constructor to prevent direct instantiation
    VehicleData() = default;

    // [SINGLETON] Delete compy constructor and assignment operator
    VehicleData(const VehicleData&) = delete;
    VehicleData& operator =(const VehicleData&) = delete;

    sensor::SbusData sbus_{};
    sensor::GPSData gps_{};

    std::atomic<uint32_t> sbus_timestamp_{0};
    std::atomic<uint32_t> gps_timestamp_{0};
};