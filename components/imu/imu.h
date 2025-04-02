#pragma once

#include <functional>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "system_types.h"
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
        int spi_clock_speed_hz{250000};
        Frequency targetFreq{Frequency::F10Hz};

        icm20948_accel_config_fs_sel_e accel_fsr{GPM_4}; // NOTE: 
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

    void updateFromConfig();

private:
    void configureSPI();
    esp_err_t configureIMU();
    esp_err_t initializeDMP();

    bool validDeviceId();
    
    static void imuTask(void* parameters);
    TaskHandle_t task_handle_{nullptr};
    esp_err_t setFullScaleRanges();

    Config config_; // TODO: rename to be config_ instead of config_t
    icm20948_device_t icm_device_{};
    spi_device_handle_t spi_handle_{nullptr};
    
    ImuData current_data_{};
    bool is_running{false};

    esp_err_t check_icm_status(icm20948_status_e status, const char* tag, const char* message);

    // SPI bus config
    spi_bus_config_t spi_bus_config_{};
    spi_device_interface_config_t device_config_{};

    static constexpr const char* TAG = "IMU";

    bool log_accel_{false};
    bool log_gyro_{false};
    bool log_freq_{false};
    // TODO: add log_quat6 and log_quat9
    // TODO: add log gyro raw
    int16_t deadband_gyro_{0};    // Stored in raw sensor units
    int16_t deadband_accel_{0};   // Stored in raw sensor units

    std::function<void()> config_callback_;


};

} // namespace sensor