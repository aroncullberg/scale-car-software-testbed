#pragma once

#include <atomic>

#include "SBUS.h"

class VehicleData {
    public:
    // [SIGNLETON] Access method (global?)
    static VehicleData& instance() {
        static VehicleData instance; // Should be thread safe (?)
        return instance;
    }

    // thread safe (i think since only one task will call it?) update methods
    void updateSBUS(const sensor::SbusData& data);

    // Unsure if this is threadsafe, should be fine since its read and not write(?)
    sensor::SbusData getSbus() const;

    uint32_t getSbusTimestamp() const {return sbus_timestamp_.load() ; }
    
    
    private:
    // [SINGLETON] private constructor to prevent direct instantiation
    VehicleData() = default;

    // [SINGLETON] Delete compy constructor and assignment operator
    VehicleData(const VehicleData&) = delete;
    VehicleData& operator =(const VehicleData&) = delete;

    sensor::SbusData sbus_{};

    std::atomic<uint32_t> sbus_timestamp_{0};
};