#include "data_pool.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


void VehicleData::updateSBUS(const sensor::SbusData& data) {
    sbus_ = data;
    sbus_timestamp_.store(xTaskGetTickCount());
}

// put more updaters here (when needed)

sensor::SbusData VehicleData::getSbus() const {
    return sbus_;
}

// put more updaters here (when needed)