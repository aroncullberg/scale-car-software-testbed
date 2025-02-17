#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensor_types.h"

extern "C" {
    #include "icm20948.h"
    #include "icm20948_spi.h"
}

static const int FOUR_MHZ = 4000000;

namespace sensor {

class IMU {
public:
    struct Config {
        spi_host_device_t spi_host{SPI3_HOST};
        int8_t spi_miso_pin{-1};
        int8_t spi_mosi_pin{-1};
        int8_t spi_sck_pin{-1};
        int8_t spi_cs_pin{-1};
        int spi_clock_speed_hz{FOUR_MHZ};
        
        icm20948_accel_config_fs_sel_e accel_fsr{GPM_16};
        icm20948_gyro_config_1_fs_sel_e gyro_fsr{DPS_500};
    };

    explicit IMU(const Config& config);
    ~IMU();

    // Delete copy operations - SPI resource can't be shared
    IMU(const IMU&) = delete;
    IMU& operator=(const IMU&) = delete;

    esp_err_t init();
    esp_err_t start();
    esp_err_t stop();

private:
    esp_err_t configureSPI();
    esp_err_t configureIMU();
    esp_err_t initializeDMP();

    bool validDeviceId();
    
    static void imuTask(void* parameters);
    TaskHandle_t task_handle_{nullptr};

    Config config_t;
    icm20948_device_t icm_device_{};
    spi_device_handle_t spi_handle_{nullptr};
    
    ImuData current_data_{};
    bool is_running{false};

    // SPI bus config
    spi_bus_config_t spi_bus_config_{};
    spi_device_interface_config_t device_config_{};

    static constexpr const char* TAG = "IMU";
};

} // namespace sensor