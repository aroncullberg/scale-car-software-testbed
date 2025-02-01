#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
    #include "icm20948.h"
    #include "icm20948_spi.h"
}

namespace sensor {

struct ImuData {
    // Raw accelerometer data (in g's)
    float accel_x{0.0f};
    float accel_y{0.0f};
    float accel_z{0.0f};
    
    // Raw gyroscope data (in deg/s)
    float gyro_x{0.0f};
    float gyro_y{0.0f};
    float gyro_z{0.0f};

    float accel_cal_x{0.0f};
    float accel_cal_y{0.0f};
    float accel_cal_z{0.0f};

    float gyro_cal_x{0.0f};
    float gyro_cal_y{0.0f};
    float gyro_cal_z{0.0f};
    
    // Quaternion orientation
    float quat_w{1.0f};  // Real component
    float quat_x{0.0f};  // i component
    float quat_y{0.0f};  // j component
    float quat_z{0.0f};  // k component
    uint16_t quat_accuracy{0}; // DMP accuracy indicator

    // game roation vector (for smooth rotations)
    float game_quat_w{1.0f};
    float game_quat_x{0.0f};
    float game_quat_y{0.0f};
    float game_quat_z{0.0f};

    // grav vector
    float linear_accel_x{0.0f};
    float linear_accel_y{0.0f};
    float linear_accel_z{0.0f};

    // Quality metrics
    struct {
        bool valid_data{false};      // Indicates if data is valid
        uint32_t error_count{0};     // Cumulative error counter
        float update_rate_hz{0.0f};  // Current update rate
        float accel_accuracy{0.0f};
        float gyro_accuracy{0.0f};
    } quality;

    // Default constructor for zero-initialization
    ImuData() = default;
};

class IMU {
public:
    // Configuration structure
    struct Config {
        // SPI Configuration
        spi_host_device_t spi_host{SPI3_HOST};
        int8_t spi_miso_pin{-1};
        int8_t spi_mosi_pin{-1};
        int8_t spi_sck_pin{-1};
        int8_t spi_cs_pin{-1};
        int spi_clock_speed_hz{7000000}; // 7MHz default
        
        // IMU Configuration
        icm20948_accel_config_fs_sel_e accel_fsr{GPM_2};
        icm20948_gyro_config_1_fs_sel_e gyro_fsr{DPS_250};
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
    
    static void imuTask(void* parameters);
    TaskHandle_t task_handle_{nullptr};

    Config config_t;
    icm20948_device_t icm_device_{};
    spi_device_handle_t spi_handle_{nullptr};
    
    ImuData current_data_{};
    bool is_running{false};

    // SPI bus config
    spi_bus_config_t bus_config_{};
    spi_device_interface_config_t device_config_{};
};

} // namespace sensor