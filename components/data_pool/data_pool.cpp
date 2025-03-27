#include "data_pool.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


void VehicleData::updateSBUS(const sensor::SbusData& data) {
    sbus_ = data;
    sbus_timestamp_.store(xTaskGetTickCount());
}

void VehicleData::updateGPS(const sensor::GpsData& data) {
    gps_ = data;
    gps_timestamp_.store(xTaskGetTickCount());
}

void VehicleData::updateIMU(const sensor::ImuData& data) {
    imu_ = data;
    imu_timestamp_.store(xTaskGetTickCount());
}

void VehicleData::updateErpm(const sensor::eRPMData& data) {
    eRPM_ = data;
}

sensor::SbusData VehicleData::getSbus() const {
    return sbus_;
}

sensor::GpsData VehicleData::getGPS() const {
    return gps_;
}

sensor::ImuData VehicleData::getImu() const {
    return imu_;
}

sensor::eRPMData VehicleData::getErpm() const
{
    return eRPM_;
}
