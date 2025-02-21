#include "imu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "data_pool.h"

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
    spi_bus_config_.miso_io_num = config_t.spi_miso_pin;
    spi_bus_config_.mosi_io_num = config_t.spi_mosi_pin;
    spi_bus_config_.sclk_io_num = config_t.spi_sck_pin;
    spi_bus_config_.quadwp_io_num = -1;
    spi_bus_config_.quadhd_io_num = -1;
    spi_bus_config_.max_transfer_sz = 512 * 8;

    device_config_.clock_speed_hz = config_t.spi_clock_speed_hz;
    device_config_.mode = 0;
    device_config_.spics_io_num = config_t.spi_cs_pin;
    device_config_.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_initialize(config_t.spi_host, &spi_bus_config_, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(config_t.spi_host, &device_config_, &spi_handle_));

    icm20948_init_spi(&icm_device_, &spi_handle_);
    
    return ESP_OK;
}

esp_err_t IMU::configureIMU() {
    constexpr int MAX_RETRIES = 3;
    int retry_count = 0;
    esp_err_t err;
    
    // while (retry_count <= MAX_RETRIES && icm20948_check_id(&icm_device_) != ICM_20948_STAT_OK) {
    while (retry_count <= MAX_RETRIES && validDeviceId()) {
        ESP_LOGW(TAG, "ID check failed, attempt %d of %d", retry_count + 1, MAX_RETRIES);
        retry_count++;
        if (retry_count < MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    if (retry_count >= MAX_RETRIES) {
        ESP_LOGE(TAG, "ID check failed after %d attempts", MAX_RETRIES);
        return ESP_ERR_NOT_FOUND;
    } else {
        ESP_LOGI(TAG, "ICM20948 check id passed");
    }

    icm20948_status_e status = ICM_20948_STAT_ERR;
    uint8_t whoami = 0x00;
    retry_count = 0;
    while (retry_count <= MAX_RETRIES && ((status != ICM_20948_STAT_OK) || (whoami != ICM_20948_WHOAMI))) {
        whoami = 0x00;
		status = icm20948_get_who_am_i(&icm_device_, &whoami);
		ESP_LOGE(TAG, "whoami does not match (0x %d). Retrying...", whoami);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    if (retry_count >= MAX_RETRIES) {
        ESP_LOGE(TAG, "whoami check failed after %d attempts, last value: 0x%02x", MAX_RETRIES, whoami);
        return ESP_ERR_NOT_FOUND;
    } else {
        ESP_LOGI(TAG, "ICM20948 whoami passed");
    } 


    icm20948_sw_reset(&icm_device_);
    // vTaskDelay(pdMS_TO_TICKS(50));
    vTaskDelay(250 / portTICK_PERIOD_MS);

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

    #if CONFIG_IMU_ENABLE_DLPF
    // DLPF settings - extra info curtesy of Claude 3.5 sonnet
    icm20948_dlpcfg_t dlp_config;

    // Accelerometer DLPF settings
    // ACC_D473BW_N499BW = Most responsive, least filtering
    // ACC_D246BW_N265BW = Good balance for RC car
    // ACC_D111BW_N136BW = More filtering, good for rough terrain
    // ACC_D50BW_N68BW   = Heavy filtering, very smooth but more lag
    dlp_config.a = ACC_D246BW_N265BW;
    
    // Gyroscope DLPF settings
    // GYR_D361BW4_N376BW5 = Most responsive, least filtering
    // GYR_D196BW6_N229BW8 = Good balance for RC car
    // GYR_D151BW8_N187BW6 = More filtering
    // GYR_D119BW5_N154BW3 = Heavy filtering, very smooth but more lag
    dlp_config.g = GYR_D196BW6_N229BW8;

    // Set DLPF configuration
    err = icm20948_set_dlpf_cfg(&icm_device_, 
        (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR), 
        dlp_config);
    if (err != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to configure DLPF settings");
        return ESP_FAIL;
    }

    // Enable DLPF for both sensors
    err = icm20948_enable_dlpf(&icm_device_, ICM_20948_INTERNAL_ACC, true);
    if (err != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to enable accelerometer DLPF");
        return ESP_FAIL;
    }

    err = icm20948_enable_dlpf(&icm_device_, ICM_20948_INTERNAL_GYR, true);
    if (err != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to enable gyroscope DLPF");
        return ESP_FAIL;
    }
    err = icm20948_enable_dlpf(&icm_device_, ICM_20948_INTERNAL_ACC, true);
    if (err != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to enable accelerometer DLPF");
        return ESP_FAIL;
    }

    err = icm20948_enable_dlpf(&icm_device_, ICM_20948_INTERNAL_GYR, true);
    if (err != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to enable gyroscope DLPF");
        return ESP_FAIL;
    }
    #endif

    // Wake up the device
    icm20948_sleep(&icm_device_, false);
    icm20948_low_power(&icm_device_, false);

    return ESP_OK;
}

bool IMU::validDeviceId() {
    return icm20948_check_id(&icm_device_) != ICM_20948_STAT_OK;
}

esp_err_t IMU::initializeDMP() {
    bool success = true;

    // Initialize DMP with default sensors
    success &= (icm20948_init_dmp_sensor_with_defaults(&icm_device_) == ICM_20948_STAT_OK);
    
    // Raw 6-axis sensors
    // success &= (inv_icm20948_enable_dmp_sensor(&icm_device_, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, 1) == ICM_20948_STAT_OK);
    // success &= (inv_icm20948_enable_dmp_sensor(&icm_device_, INV_ICM20948_SENSOR_RAW_GYROSCOPE, 1) == ICM_20948_STAT_OK);
    
    // Calibrated 6 axis sensors
    success &= (inv_icm20948_enable_dmp_sensor(&icm_device_, INV_ICM20948_SENSOR_ACCELEROMETER, 1) == ICM_20948_STAT_OK);
    success &= (inv_icm20948_enable_dmp_sensor(&icm_device_, INV_ICM20948_SENSOR_GYROSCOPE, 1) == ICM_20948_STAT_OK);

    //  - Game Rotation Vector (good for fast motion, no magnetometer)
    success &= (inv_icm20948_enable_dmp_sensor(&icm_device_, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 1) == ICM_20948_STAT_OK);
    //  - Rotation Vector (includes magnetometer for drift correction - slower, more accurate)
    success &= (inv_icm20948_enable_dmp_sensor(&icm_device_, INV_ICM20948_SENSOR_ROTATION_VECTOR, 1) == ICM_20948_STAT_OK);

    // Derived motion data
    success &= (inv_icm20948_enable_dmp_sensor(&icm_device_, INV_ICM20948_SENSOR_LINEAR_ACCELERATION, 1) == ICM_20948_STAT_OK);

    // Set DMP output rate for fast sensors (accel, gyro, mag(maybe later))
    success &= (inv_icm20948_set_dmp_sensor_period(&icm_device_, DMP_ODR_Reg_Accel, 0) == ICM_20948_STAT_OK);
    success &= (inv_icm20948_set_dmp_sensor_period(&icm_device_, DMP_ODR_Reg_Gyro, 0) == ICM_20948_STAT_OK);
    success &= (inv_icm20948_set_dmp_sensor_period(&icm_device_, DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_STAT_OK);

    // Quaternion rates
    // Quat6 - maximum rate for what we for the fast one
    success &= (inv_icm20948_set_dmp_sensor_period(&icm_device_, DMP_ODR_Reg_Quat6, 0) == ICM_20948_STAT_OK);
    // Slower rate (for ROTATION_VECTOR) - Not needed with Game Rotation Vector
    success &= (inv_icm20948_set_dmp_sensor_period(&icm_device_, DMP_ODR_Reg_Quat9, 4) == ICM_20948_STAT_OK);   


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
        imuTask,
        "imu_task",
        4096,
        this,
        5,
        &task_handle_
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
            bool data_updated = false;
            
            // Look, contrary to the name i think this is gravity compensated accel data since we turned on that before
            if (dmp_data.header & DMP_header_bitmap_Accel) {
                instance->current_data_.accel_x = dmp_data.Raw_Accel.Data.X;
                instance->current_data_.accel_y = dmp_data.Raw_Accel.Data.Y;
                instance->current_data_.accel_z = dmp_data.Raw_Accel.Data.Z;
                data_updated = true;

                #if CONFIG_IMU_LOG_ACCEL
                // Convert to g (1 unit = 1/2048 g)
                ESP_LOGI(TAG, "Accel: x=%.3f g, y=%.3f g, z=%.3f g",
                    (static_cast<double>(instance->current_data_.accel_x) / 2048.0) * 9.82,
                    (static_cast<double>(instance->current_data_.accel_y) / 2048.0) * 9.82,
                    (static_cast<double>(instance->current_data_.accel_z) / 2048.0) * 9.82);
           
                #endif
            }

            if (dmp_data.header & DMP_header_bitmap_Gyro) {
                instance->current_data_.gyro_x = dmp_data.Raw_Gyro.Data.X;
                instance->current_data_.gyro_y = dmp_data.Raw_Gyro.Data.Y;
                instance->current_data_.gyro_z = dmp_data.Raw_Gyro.Data.Z;
                data_updated = true;
                #if CONFIG_IMU_LOG_GYRO
                    ESP_LOGI(TAG, "Gyro: x=%.3f dps, y=%.3f dps, z=%.3f dps",
                        static_cast<double>(instance->current_data_.gyro_x) / 64.0,
                        static_cast<double>(instance->current_data_.gyro_y) / 64.0,
                        static_cast<double>(instance->current_data_.gyro_z) / 64.0);
                #endif
            } 

            if (dmp_data.header & DMP_header_bitmap_Gyro_Calibr) {
                instance->current_data_.gyro_cal_x = dmp_data.Gyro_Calibr.Data.X;
                instance->current_data_.gyro_cal_y = dmp_data.Gyro_Calibr.Data.Y;
                instance->current_data_.gyro_cal_z = dmp_data.Gyro_Calibr.Data.Z;
                data_updated = true;
                #if CONFIG_IMU_LOG_CALIBGYRO
                    ESP_LOGI(TAG, "GyroCalib: x=%.3f dps, y=%.3f dps, z=%.3f dps",
                        static_cast<double>(instance->current_data_.gyro_cal_x) / 64.0,
                        static_cast<double>(instance->current_data_.gyro_cal_y) / 64.0,
                        static_cast<double>(instance->current_data_.gyro_cal_z) / 64.0);
                #endif
            } 

            // Process 6-axis quaternion first (primary orientaiton source)
            // Process calibrated gyroscope data
            if (dmp_data.header & DMP_header_bitmap_Quat6) {
                instance->current_data_.quat6_x = dmp_data.Quat6.Data.Q1;
                instance->current_data_.quat6_y = dmp_data.Quat6.Data.Q2;
                instance->current_data_.quat6_z = dmp_data.Quat6.Data.Q3;
                data_updated = true;
                #if CONFIG_IMU_LOG_QUAT6
                    // Convert to quaternion units (Q30 format, so divide by 2^30)
                    ESP_LOGI(TAG, "Quat6: x=%.6f, y=%.6f, z=%.6f",
                        static_cast<double>(instance->current_data_.quat6_x) / 1073741824.0,
                        static_cast<double>(instance->current_data_.quat6_y) / 1073741824.0,
                        static_cast<double>(instance->current_data_.quat6_z) / 1073741824.0);
                #endif
            } 

            // Process 9-axis quaternion for drift correction
            if (dmp_data.header & DMP_header_bitmap_Quat9) {
                instance->current_data_.quat9_x = dmp_data.Quat9.Data.Q1;
                instance->current_data_.quat9_y = dmp_data.Quat9.Data.Q2;
                instance->current_data_.quat9_z = dmp_data.Quat9.Data.Q3;
                instance->current_data_.quat9_accuracy = dmp_data.Quat9.Data.Accuracy;
                data_updated = true;
                #if CONFIG_IMU_LOG_QUAT9
                // Convert to quaternion units (Q30 format, so divide by 2^30)
                ESP_LOGI(TAG, "Quat9: x=%.6f, y=%.6f, z=%.6f, acc=%u",
                    static_cast<double>(instance->current_data_.quat9_x) / 1073741824.0,
                    static_cast<double>(instance->current_data_.quat9_y) / 1073741824.0,
                    static_cast<double>(instance->current_data_.quat9_z) / 1073741824.0,
                    instance->current_data_.quat9_accuracy);
                #endif
            } 

            if (data_updated) {
                instance->current_data_.quality.valid_data = true;

                //caluclate update rate
                TickType_t current_time = xTaskGetTickCount();
                float interval = static_cast<float>(current_time - last_wake_time) * portTICK_PERIOD_MS;
                if (interval > 0) {
                    instance->current_data_.quality.update_rate_hz = 1000.0f / interval;
                }
                last_wake_time = current_time;

                VehicleData::instance().updateIMU(instance->current_data_);
            }
        } else if (status == ICM_20948_STAT_ERR) {
            instance->current_data_.quality.error_count++;
            ESP_LOGW(TAG, "DMP read error, count %lu", instance->current_data_.quality.error_count);
        }

        // unly delai if no data is aviablle
        if (status != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}



} // namespace sensor