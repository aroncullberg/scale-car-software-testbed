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
    constexpr int MAX_RETRIES = 30;
    int retry_count = 0;
    
    // ID
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
    }
    ESP_LOGI(TAG, "ICM20948 check id passed");

    // WHOAMI
    icm20948_status_e status = ICM_20948_STAT_ERR;
    uint8_t whoami = 0x00;
    retry_count = 0;
    while (retry_count <= MAX_RETRIES && ((status != ICM_20948_STAT_OK) || (whoami != ICM_20948_WHOAMI))) {
        whoami = 0x00;
        status = icm20948_get_who_am_i(&icm_device_, &whoami);
        if (status != ICM_20948_STAT_OK || whoami != ICM_20948_WHOAMI) {
            ESP_LOGW(TAG, "whoami does not match (0x%02x). Retrying...", whoami);
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            break;
        }
    }

    if (retry_count >= MAX_RETRIES) {
        ESP_LOGE(TAG, "whoami check failed after %d attempts, last value: 0x%02x", MAX_RETRIES, whoami);
        return ESP_ERR_NOT_FOUND;
    } else {
        ESP_LOGI(TAG, "ICM20948 whoami passed: 0x%02x", whoami);
    }

    // Reste the device
    icm20948_sw_reset(&icm_device_);
    vTaskDelay(250 / portTICK_PERIOD_MS);

    // Wake up the device before configuring
    // NOTE: this is kinda important, i mean what moron would try to configure it without waking up the device FIRST.
    icm20948_sleep(&icm_device_, false);
    vTaskDelay(pdMS_TO_TICKS(10));
    icm20948_low_power(&icm_device_, false);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set full scale ranges using direct register access, wont work otherwise, i dont like it either.
    esp_err_t err = setFullScaleRanges();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set full scale ranges");
        return err;
    }

    // Configure sensors for continuous mode
    constexpr auto sensors = static_cast<icm20948_internal_sensor_id_bm>(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);

    if (icm20948_set_sample_mode(&icm_device_, sensors, SAMPLE_MODE_CONTINUOUS) != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to set sample mode");
        return ESP_FAIL;
    }

    #if CONFIG_IMU_ENABLE_DLPF
    icm20948_dlpcfg_t dlp_config;

    // ACC_D473BW_N499BW = Most responsive, least filtering
    // ACC_D246BW_N265BW = Good balance for RC car
    // ACC_D111BW_N136BW = More filtering, good for rough terrain
    // ACC_D50BW_N68BW   = Heavy filtering, very smooth but more lag
    dlp_config.a = ACC_D246BW_N265BW;
    
    // GYR_D361BW4_N376BW5 = Most responsive, least filtering
    // GYR_D196BW6_N229BW8 = Good balance for RC car
    // GYR_D151BW8_N187BW6 = More filtering
    // GYR_D119BW5_N154BW3 = Heavy filtering, very smooth but more lag
    dlp_config.g = GYR_D196BW6_N229BW8;

    // Set DLPF configuration
    status = icm20948_set_dlpf_cfg(&icm_device_, 
        (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR), 
        dlp_config);
    if (status != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to configure DLPF settings");
        return ESP_FAIL;
    }

    // Enable DLPF for both sensors
    status = icm20948_enable_dlpf(&icm_device_, ICM_20948_INTERNAL_ACC, true);
    if (status != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to enable accelerometer DLPF");
        return ESP_FAIL;
    }

    status = icm20948_enable_dlpf(&icm_device_, ICM_20948_INTERNAL_GYR, true);
    if (status != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to enable gyroscope DLPF");
        return ESP_FAIL;
    }
    #endif

    return ESP_OK;
}

esp_err_t IMU::setFullScaleRanges() {
    #if CONFIG_IMU_DEBUG_FSR
    // Get current settings for debug purposes
    icm20948_set_bank(&icm_device_, 2);
    
    icm20948_accel_config_t accel_config_before;
    icm20948_status_e acc_status = icm20948_execute_r(&icm_device_, AGB2_REG_ACCEL_CONFIG, 
                                                     (uint8_t*)&accel_config_before, 
                                                     sizeof(accel_config_before));
    
    icm20948_gyro_config_1_t gyro_config_before;
    icm20948_status_e gyro_status = icm20948_execute_r(&icm_device_, AGB2_REG_GYRO_CONFIG_1, 
                                                      (uint8_t*)&gyro_config_before, 
                                                      sizeof(gyro_config_before));
    
    if (acc_status == ICM_20948_STAT_OK && gyro_status == ICM_20948_STAT_OK) {
        ESP_LOGI(TAG, "Current settings - Accel FSR: %d, Gyro FSR: %d", 
                 accel_config_before.ACCEL_FS_SEL, gyro_config_before.GYRO_FS_SEL);
    }
    #endif
    
    // Make sure we're in the right bank
    icm20948_set_bank(&icm_device_, 2);
    
    // Set accelerometer FSR
    icm20948_accel_config_t accel_config;
    icm20948_status_e status = icm20948_execute_r(&icm_device_, AGB2_REG_ACCEL_CONFIG, 
                                                (uint8_t*)&accel_config, 
                                                sizeof(accel_config));
    if (status != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer config");
        return ESP_FAIL;
    }
    
    accel_config.ACCEL_FS_SEL = config_t.accel_fsr;
    status = icm20948_execute_w(&icm_device_, AGB2_REG_ACCEL_CONFIG, 
                               (uint8_t*)&accel_config, 
                               sizeof(accel_config));
    if (status != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to write accelerometer FSR");
        return ESP_FAIL;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Give it time to apply
    
    // Set gyroscope FSR
    icm20948_gyro_config_1_t gyro_config;
    status = icm20948_execute_r(&icm_device_, AGB2_REG_GYRO_CONFIG_1, 
                               (uint8_t*)&gyro_config, 
                               sizeof(gyro_config));
    if (status != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope config");
        return ESP_FAIL;
    }
    
    gyro_config.GYRO_FS_SEL = config_t.gyro_fsr;
    status = icm20948_execute_w(&icm_device_, AGB2_REG_GYRO_CONFIG_1, 
                               (uint8_t*)&gyro_config, 
                               sizeof(gyro_config));
    if (status != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "Failed to write gyroscope FSR");
        return ESP_FAIL;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Give it time to apply
    
    #if CONFIG_IMU_DEBUG_FSR
    // Verify the settings were correctly applied
    icm20948_set_bank(&icm_device_, 2);
    
    icm20948_accel_config_t accel_config_after;
    icm20948_execute_r(&icm_device_, AGB2_REG_ACCEL_CONFIG, 
                      (uint8_t*)&accel_config_after, 
                      sizeof(accel_config_after));
    
    icm20948_gyro_config_1_t gyro_config_after;
    icm20948_execute_r(&icm_device_, AGB2_REG_GYRO_CONFIG_1, 
                       (uint8_t*)&gyro_config_after, 
                       sizeof(gyro_config_after));
    
    ESP_LOGI(TAG, "After setting - Accel FSR: %d (expected %d), Gyro FSR: %d (expected %d)", 
             accel_config_after.ACCEL_FS_SEL, config_t.accel_fsr,
             gyro_config_after.GYRO_FS_SEL, config_t.gyro_fsr);
    #endif
    
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
    success &= (inv_icm20948_enable_dmp_sensor(&icm_device_, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, 1) == ICM_20948_STAT_OK);


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

    BaseType_t task_created = xTaskCreatePinnedToCore(
        imuTask,
        "imu_task",
        4096,
        this,
        5,
        &task_handle_,
        1
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
    const auto instance = static_cast<IMU*>(parameters);
    TickType_t last_wake_time = xTaskGetTickCount();
    icm_20948_DMP_data_t dmp_data;

    while (true) {
        icm20948_status_e status = inv_icm20948_read_dmp_data(&instance->icm_device_, &dmp_data);
        
        if ((status == ICM_20948_STAT_OK) || (status == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL)) {
            bool data_updated = false;
            
            // Look, contrary to the name i think this is gravity compensated accel data since we turned on that before
            if (dmp_data.header & DMP_header_bitmap_Accel) {
                instance->current_data_.accel_x = dmp_data.Raw_Accel.Data.X;
                instance->current_data_.accel_y = -dmp_data.Raw_Accel.Data.Y;
                instance->current_data_.accel_z = dmp_data.Raw_Accel.Data.Z;
                data_updated = true;

                #if CONFIG_IMU_LOG_ACCEL
                // Convert to m/s (1 unit = 1/2048 g)[m/s^2]
                ESP_LOGI(TAG, "Accel: x=%4.3f g, y=%4.3f g, z=%4.3f g",
                    (static_cast<double>(instance->current_data_.accel_x) / 8192.0f),
                    (static_cast<double>(instance->current_data_.accel_y) / 8192.0f),
                    (static_cast<double>(instance->current_data_.accel_z) / 8192.0f));
                #endif
            }

            if (dmp_data.header & DMP_header_bitmap_Gyro) {
                instance->current_data_.gyro_x = dmp_data.Raw_Gyro.Data.X - dmp_data.Raw_Gyro.Data.BiasX;
                instance->current_data_.gyro_y = dmp_data.Raw_Gyro.Data.Y - dmp_data.Raw_Gyro.Data.BiasY;
                instance->current_data_.gyro_z = dmp_data.Raw_Gyro.Data.Z - dmp_data.Raw_Gyro.Data.BiasZ;
                data_updated = true;

                // TODO: Replace this with a keystore value such that you can say log accel data for x seconds and then it turns off.
                #if CONFIG_IMU_LOG_GYRO
                    ESP_LOGI(TAG, "Gyro: x=%.3f dps, y=%.3f dps, z=%.3f dps",
                        static_cast<double>(instance->current_data_.gyro_x) / 65.5f,
                        static_cast<double>(instance->current_data_.gyro_y) / 65.5f,
                        static_cast<double>(instance->current_data_.gyro_z) / 65.5f);


                    // ESP_LOGI(TAG, "Gyro Bias Values: X=%d, Y=%d, Z=%d", 
                    //             dmp_data.Raw_Gyro.Data.BiasX,
                    //             dmp_data.Raw_Gyro.Data.BiasY, 
                    //             dmp_data.Raw_Gyro.Data.BiasZ);
                        
                #endif
            } 

            if (dmp_data.header & DMP_header_bitmap_Quat6) {
                instance->current_data_.quat6_x = dmp_data.Quat6.Data.Q1;
                instance->current_data_.quat6_y = -dmp_data.Quat6.Data.Q2;
                instance->current_data_.quat6_z = dmp_data.Quat6.Data.Q3;
                data_updated = true;
                #if CONFIG_IMU_LOG_QUAT6
                    // Convert to quaternion units (Q30 format, so divide by 2^30)
                    ESP_LOGI(TAG, "Quat6: x=%3.6f, y=%3.6f, z=%3.6f",
                        static_cast<double>(instance->current_data_.quat6_x) / 1073741824.0,
                        static_cast<double>(instance->current_data_.quat6_y) / 1073741824.0,
                        static_cast<double>(instance->current_data_.quat6_z) / 1073741824.0);
                #endif
            }

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
