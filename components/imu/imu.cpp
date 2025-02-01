#include "imu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "data_pool.h"

static const char* TAG = "IMU";

namespace sensor {

IMU::IMU(const Config& config) : config_t(config) {
    ESP_LOGI(TAG, "IMU instance created");
}

IMU::~IMU() {
    stop();
    ESP_LOGI(TAG, "IMU instance destroyed");
}

esp_err_t IMU::init() {
    ESP_LOGI(TAG, "Initializing IMU on SPI bus (MISO:%d, MOSI:%d, SCK:%d, CS:%d)",
             config_t.spi_miso_pin, config_t.spi_mosi_pin, 
             config_t.spi_sck_pin, config_t.spi_cs_pin);
             
    esp_err_t err = configureSPI();
    if (err != ESP_OK) {
        return err;
    }

    err = configureIMU();
    if (err != ESP_OK) {
        return err;
    }

    return initializeDMP();
}

esp_err_t IMU::configureSPI() {
    // Configure SPI bus
    bus_config_.miso_io_num = config_t.spi_miso_pin;
    bus_config_.mosi_io_num = config_t.spi_mosi_pin;
    bus_config_.sclk_io_num = config_t.spi_sck_pin;
    bus_config_.quadwp_io_num = -1;
    bus_config_.quadhd_io_num = -1;
    bus_config_.max_transfer_sz = 4096;

    // Configure device settings
    device_config_.clock_speed_hz = config_t.spi_clock_speed_hz;
    device_config_.mode = 0;
    device_config_.spics_io_num = config_t.spi_cs_pin;
    device_config_.queue_size = 1;

    // Initialize bus and add device
    esp_err_t err = spi_bus_initialize(config_t.spi_host, &bus_config_, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialization failed");
        return err;
    }

    err = spi_bus_add_device(config_t.spi_host, &device_config_, &spi_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI device addition failed");
        return err;
    }

    // Initialize ICM20948 with SPI interface
    icm20948_init_spi(&icm_device_, &spi_handle_);
    
    return ESP_OK;
}

esp_err_t IMU::configureIMU() {
    // Check device ID
    if (icm20948_check_id(&icm_device_) != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "ICM20948 ID check failed");
        return ESP_FAIL;
    }

    // Perform software reset
    icm20948_sw_reset(&icm_device_);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Configure sensors for continuous mode
    icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(
        ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR
    );
    
    if (icm20948_set_sample_mode(&icm_device_, sensors, SAMPLE_MODE_CONTINUOUS) != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to set sample mode");
        return ESP_FAIL;
    }

    // Set full scale ranges
    icm20948_fss_t fsr;
    fsr.a = config_t.accel_fsr;
    fsr.g = config_t.gyro_fsr;
    if (icm20948_set_full_scale(&icm_device_, sensors, fsr) != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to set full scale ranges");
        return ESP_FAIL;
    }

    // Wake up the device
    icm20948_sleep(&icm_device_, false);
    icm20948_low_power(&icm_device_, false);

    return ESP_OK;
}

esp_err_t IMU::initializeDMP() {
    bool success = true;

    // Initialize DMP
    success &= (icm20948_init_dmp_sensor_with_defaults(&icm_device_) == ICM_20948_STAT_OK);
    
    // Enable orientation sensor
    success &= (inv_icm20948_enable_dmp_sensor(&icm_device_, INV_ICM20948_SENSOR_ORIENTATION, 1) == ICM_20948_STAT_OK);
    
    // Set maximum output rate
    success &= (inv_icm20948_set_dmp_sensor_period(&icm_device_, DMP_ODR_Reg_Quat9, 0) == ICM_20948_STAT_OK);

    // Enable FIFO and DMP
    success &= (icm20948_enable_fifo(&icm_device_, true) == ICM_20948_STAT_OK);
    success &= (icm20948_enable_dmp(&icm_device_, true) == ICM_20948_STAT_OK);
    
    // Reset DMP and FIFO
    success &= (icm20948_reset_dmp(&icm_device_) == ICM_20948_STAT_OK);
    success &= (icm20948_reset_fifo(&icm_device_) == ICM_20948_STAT_OK);

    if (!success) {
        ESP_LOGE(TAG, "DMP initialization failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "DMP initialized successfully");
    return ESP_OK;
}

esp_err_t IMU::start() {
    if (is_running) {
        ESP_LOGW(TAG, "IMU already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t task_created = xTaskCreate(
        imuTask,                    // Task function
        "imu_task",                 // Task name
        4096, // Stack size (from Kconfig) // TODO: usekfonfig
        this,                       // Task parameter
        5,   // Task priority (from Kconfig) // TODO: usekfonfig
        &task_handle_              // Task handle
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return ESP_ERR_NO_MEM;
    }

    is_running = true;
    ESP_LOGI(TAG, "IMU Started");
    return ESP_OK;
}

esp_err_t IMU::stop() {
    if (!is_running) {
        return ESP_OK;
    }

    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    is_running = false;
    ESP_LOGI(TAG, "IMU stopped");
    return ESP_OK;
}

void IMU::imuTask(void* parameters) {
    IMU* instance = static_cast<IMU*>(parameters);
    TickType_t last_wake_time = xTaskGetTickCount();
    icm_20948_DMP_data_t dmp_data;

    while (true) {
        icm20948_status_e status = inv_icm20948_read_dmp_data(&instance->icm_device_, &dmp_data);
        
        if ((status == ICM_20948_STAT_OK) || (status == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL)) {
            // Check if we have quaternion data
            if ((dmp_data.header & DMP_header_bitmap_Quat9) > 0) {
                // Scale quaternion data (comes in Q30 format)
                float q1 = ((float)dmp_data.Quat9.Data.Q1) / 1073741824.0f;
                float q2 = ((float)dmp_data.Quat9.Data.Q2) / 1073741824.0f;
                float q3 = ((float)dmp_data.Quat9.Data.Q3) / 1073741824.0f;
                float q0 = sqrt(1.0f - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                // Update current data
                instance->current_data_.quat_w = q0;
                instance->current_data_.quat_x = q1;
                instance->current_data_.quat_y = q2;
                instance->current_data_.quat_z = q3;
                instance->current_data_.quat_accuracy = dmp_data.Quat9.Data.Accuracy;
                instance->current_data_.quality.valid_data = true;
                ESP_LOGI(TAG, "%2.3f %2.3f %2.3f %2.3f", q0, q1, q2, q3);

                // Calculate update rate
                TickType_t current_time = xTaskGetTickCount();
                float interval = (float)(current_time - last_wake_time) * portTICK_PERIOD_MS;
                if (interval > 0) {
                    instance->current_data_.quality.update_rate_hz = 1000.0f / interval;
                }
                last_wake_time = current_time;

                // Update global data pool
                VehicleData::instance().updateIMU(instance->current_data_);
            }
        } else if (status == ICM_20948_STAT_ERR) {
            instance->current_data_.quality.error_count++;
        }

        // Only delay if no more data is available
        if (status != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
            vTaskDelay(pdMS_TO_TICKS(1)); // 1ms delay
        }
    }
}

} // namespace sensor