#include "DataManager.h"
#include "esp_log.h"
#include "data_pool.h"

DataManager::DataManager(const Config& config) {
    DataManager::config_ = config;
    ESP_LOGI(TAG, "Data manage instance created");
}

DataManager::~DataManager(){
    stop();
    ESP_LOGI(TAG, "Data manager instance destoryed");
}

esp_err_t DataManager::start() {
    if (is_running_) {
        ESP_LOGW(TAG, "Data manager already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t task_created = xTaskCreate(
        dataTask,
        "data_manager",
        4096, // WARNING: kconfig
        this,
        5, // WARNING: kconfig
        &task_handle_
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create data manager task");
        return ESP_ERR_NO_MEM;
    }

    is_running_ = true;
    ESP_LOGI(TAG, "Data manager started");
    return ESP_OK;
}

esp_err_t DataManager::stop() {
    if (!is_running_) {
        return ESP_OK;
    }

    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    is_running_ = false;
    ESP_LOGI(TAG, "data manager stopped");
    return ESP_OK;
}

void DataManager::dataTask(void* parameters) {
    DataManager* instance = static_cast<DataManager*>(parameters);
    TickType_t last_wake_time = xTaskGetTickCount();

    while(true) {
        instance->logSbusData();
        instance->logGpsData();
        instance->logImuData();
        // instance->checkSensorHealth();
        ESP_LOGI(TAG, "");

        vTaskDelayUntil(&last_wake_time, instance->config_.task_period);
    }
}

void DataManager::logSbusData() {
    auto& data_pool = VehicleData::instance();
    const auto sbus_data = data_pool.getSbus();
    const auto timestamp = data_pool.getSbusTimestamp();

    // Only log if we have new data
    // if (timestamp == sensor_timestamps_.last_sbus) {
    //     return;
    // }
    sensor_timestamps_.last_sbus = timestamp;

    // Log control channels
    ESP_LOGI(TAG, "SBUS [%lu] Control: Throttle=%.2f, Steering=%.2f", 
             timestamp,
             sbus_data.channels[0],  // Throttle
             sbus_data.channels[1]); // Steering

    // Log quality metrics
    ESP_LOGI(TAG, "SBUS Quality: Loss=%d%%, Rate=%.1fHz, Valid=%d", 
             sbus_data.quality.frame_loss_percent,
             1000.0f / sbus_data.quality.frame_interval_ms,
             sbus_data.quality.valid_signal);
}

void DataManager::logGpsData() {
    auto& data_pool = VehicleData::instance();
    const auto gps_data = data_pool.getGPS();
    const auto timestamp = data_pool.getGPSTimestamp();

    // Only log if we have new data
    // if (timestamp == sensor_timestamps_.last_gps) {
    //     return;
    // }
    sensor_timestamps_.last_gps = timestamp;

    // Log position data
    ESP_LOGI(TAG, "GPS [%lu] Pos: Lat=%ld.%07d, Lon=%ld.%07d, Alt=%.2fm", 
             timestamp,
             gps_data.latitude / 10000000, static_cast<int>(abs(gps_data.latitude % 10000000)),
             gps_data.longitude / 10000000, static_cast<int>(abs(gps_data.longitude % 10000000)),
             gps_data.altitude_mm / 1000.0f);

    // Log speed and course
    if (gps_data.speed_valid) {
        ESP_LOGI(TAG, "GPS Speed: %.1f m/s, Course: %.1f°", 
                 gps_data.speed_mmps / 1000.0f,
                 gps_data.ground_course / 100.0f);
    }

    // Log quality metrics
    ESP_LOGI(TAG, "GPS Quality: Fix=%d, Sats=%d, HDOP=%.1f", 
             gps_data.quality.fix_type,
             gps_data.quality.satellites,
             gps_data.quality.hdop / 100.0f);
}

void DataManager::logImuData() {
    auto& data_pool = VehicleData::instance();
    const auto imu_data = data_pool.getImu();
    const auto timestamp = data_pool.getImuTimestamp();

    // Only log if we have new data
    // if (timestamp == sensor_timestamps_.last_imu) {
    //     return;
    // }
    sensor_timestamps_.last_imu = timestamp;

    // Log acceleration data
    ESP_LOGI(TAG, "IMU [%lu] Accel (g): X=%.2f, Y=%.2f, Z=%.2f", 
             timestamp,
             imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);

    // Log gyroscope data
    ESP_LOGI(TAG, "IMU Gyro (deg/s): X=%.2f, Y=%.2f, Z=%.2f",
             imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);

    // Log orientation quaternion
    ESP_LOGI(TAG, "IMU Quat: W=%.3f, X=%.3f, Y=%.3f, Z=%.3f",
             imu_data.quat_w, imu_data.quat_x, imu_data.quat_y, imu_data.quat_z);

    // Log quality metrics
    ESP_LOGI(TAG, "IMU Quality: Rate=%.1fHz, Errors=%ld", 
             imu_data.quality.update_rate_hz,
             imu_data.quality.error_count);
}

void DataManager::checkSensorHealth() {
    auto& data_pool = VehicleData::instance();
    const uint32_t current_time = xTaskGetTickCount();
    const uint32_t timeout = pdMS_TO_TICKS(1000); // 1 second timeout

    // Check each sensor's last update time
    bool sbus_healthy = (current_time - data_pool.getSbusTimestamp()) < timeout;
    bool gps_healthy = (current_time - data_pool.getGPSTimestamp()) < timeout;
    bool imu_healthy = (current_time - data_pool.getImuTimestamp()) < timeout;

    // Log only if there are issues
    if (!sbus_healthy || !gps_healthy || !imu_healthy) {
        ESP_LOGW(TAG, "Sensor Health: SBUS=%d, GPS=%d, IMU=%d",
                 sbus_healthy, gps_healthy, imu_healthy);
    }
}
