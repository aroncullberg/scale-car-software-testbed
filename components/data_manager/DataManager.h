#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class DataManager {
public:
    struct Config {
        TickType_t task_period{pdMS_TO_TICKS(100)};
    };

    explicit DataManager(const Config& config);
    ~DataManager();

    // delete copy operations
    DataManager(const DataManager&) = delete;
    DataManager& operator=(const DataManager&) = delete;

    esp_err_t start();
    esp_err_t stop();
private:
    static constexpr const char* TAG = "DataManager";

    static void dataTask(void* paramters);
    TaskHandle_t task_handle_{nullptr};

    Config config_;
    bool is_running_{false};

    void logSbusData();
    void logGpsData();
    void logImuData();
    void checkSensorHealth();

    struct {
        uint32_t last_sbus{0};
        uint32_t last_gps{0};
        uint32_t last_imu{0};
    } sensor_timestamps_;
};