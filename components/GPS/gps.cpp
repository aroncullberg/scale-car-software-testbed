#include <cstring>
#include <algorithm>
#include "gps.h"
#include "esp_log.h"
#include <data_pool.h>
#include <cinttypes>

namespace sensor {

GPS::GPS(const Config& config) {
    config_t = config;
    ESP_LOGI(TAG, "GPS instance created");
}

GPS::~GPS() {
    stop();
    ESP_LOGI(TAG, "GPS instance destroyed");
}

esp_err_t GPS::init() {
    // ESP_LOGI(TAG, "initializing GPS on UART%d (TX:%d, RX:%d, baud:%d)",
            //  config_t.uart_num, config_t.uart_tx_pin, config_t.uart_rx_pin, config_t.baud_rate);
    // In your init() function, add:
    ESP_LOGI(TAG, "GPS Module Info:");
    ESP_LOGI(TAG, "  UART Num: %d", config_t.uart_num);
    ESP_LOGI(TAG, "  RX Pin: %d", config_t.uart_rx_pin);
    ESP_LOGI(TAG, "  Baud Rate: %d", config_t.baud_rate);
    
    return configureUART();
}

esp_err_t GPS::configureUART() {
    uart_config_t uart_config = {
        .baud_rate = config_t.baud_rate,  // Explicitly set to 9600
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,  // Try explicit clock source
    };

    esp_err_t err = uart_param_config(config_t.uart_num, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed (%d)", err);
        return err;
    }

    // Set pins with explicit pull settings
    err = uart_set_pin(config_t.uart_num,
                      config_t.uart_tx_pin,
                      config_t.uart_rx_pin,
                      GPIO_NUM_NC,  // No RTS
                      GPIO_NUM_NC); // No CTS
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed (%d)", err);
        return err;
    }

    // Install driver with larger buffers
    err = uart_driver_install(config_t.uart_num, 
                            config_t.rx_buffer_size,  // Larger RX buffer
                            config_t.tx_buffer_size,         // No TX buffer
                            0,         // No event queue
                            nullptr,      // No queue handle
                            0);        // No interrupt flags
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed (%d)", err);
        return err;
    }

    // Send UBX command to disable UBX protocol
    const auto ubx_cfg_prt = "\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x00\xC2\x01\x00\x07\x00\x01\x00\x00\x00\x00\x00\xC0\x7E";
    uart_write_bytes(config_t.uart_num, ubx_cfg_prt, 28);  // Exact 28-byte UBX packet

    // const char* ubx_factory_reset = 
    //     "\xB5\x62\x06\x09\x0D\x00\xFF\xFF\x00\x00\x00\x00\x00\x00\xFF\xFF\x00\x00\x17\x2B\x7E";

    // // In GPS::configureUART(), after initializing UART:
    // uart_write_bytes(config_t.uart_num, ubx_factory_reset, strlen(ubx_factory_reset));
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for command to process




    return ESP_OK;
}

esp_err_t GPS::start() {
    if (is_running) {
        ESP_LOGW(TAG, "GPS already active");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t task_created = xTaskCreatePinnedToCore(
        gpsTask,                           // task function
        "gps_task",                        // task name
        4096,        // stack size (Kconfig)
        this,                               // ???
        5,          // Task priority (Kconfig)
        &task_handle_,                       // self explanatory 
        1
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPS task");
        return ESP_ERR_NO_MEM;
    }

    is_running = true;
    ESP_LOGI(TAG, "GPS Started");
    return ESP_OK;
}

esp_err_t GPS::stop() {
    if (!is_running) {
        return ESP_OK;
    }

    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    is_running = false;
    ESP_LOGI(TAG, "GPS stopped");
    return ESP_OK;
}

void GPS::gpsTask(void* parameters) {
    const auto gps = static_cast<GPS*>(parameters);
    const uart_port_t uart_num = gps->config_t.uart_num;
    
    // Buffer for reading UART data
    uint8_t data[512];  // Smaller buffer since we process more frequently
    
    ESP_LOGI(TAG, "GPS task started");
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (true) {
        // Read available UART data
        size_t buffered_length;
        uart_get_buffered_data_len(uart_num, &buffered_length);
        
        if (buffered_length > 0) {
            int read_length = uart_read_bytes(uart_num, 
                                           data, 
                                           std::min(static_cast<size_t>(buffered_length), 
                                                    static_cast<size_t>(sizeof(data))),
                                           0);  // No waiting
            
            if (read_length > 0) {
                // ESP_LOGI(TAG, "%s", data);
                // Feed data to TinyGPS++
                for (int i = 0; i < read_length; i++) {
                    gps->tiny_gps_.encode(data[i]);
                }
                gps->processGPSData(); 
            }
        }
        
        // Run at fixed frequency (10Hz default)
        vTaskDelayUntil(&last_wake_time, gps->config_t.task_period);
    }
}


void GPS::processGPSData() {
    if (tiny_gps_.location.isValid() && tiny_gps_.location.isUpdated()) {
        GpsData data;
        
        // Position data (unchanged)
        data.latitude = tiny_gps_.location.lat() * 10000000;  // Convert to fixed point
        data.longitude = tiny_gps_.location.lng() * 10000000;
        data.altitude_mm = tiny_gps_.altitude.meters() * 1000;
        
        // Speed data (new)
        if (tiny_gps_.speed.isValid()) {
            // Convert from knots to m/s and store as mm/s for fixed point
            data.speed_mmps = tiny_gps_.speed.mps() * 1000;  // Need to add this to GpsData struct
            data.ground_course = tiny_gps_.course.deg() * 100; // Store as centidegrees
            data.speed_valid = true;
        } else {
            data.speed_mmps = 0;
            data.ground_course = 0;
            data.speed_valid = false;
        }
        
        // Quality data (unchanged)
        data.quality.fix_type = tiny_gps_.location.isValid() ? 
            (tiny_gps_.altitude.isValid() ? 2 : 1) : 0;
        data.quality.satellites = tiny_gps_.satellites.value();
        data.quality.hdop = tiny_gps_.hdop.value() * 100;  // Convert to fixed point
        
        // Status (unchanged)
        data.status.bits.valid_fix = tiny_gps_.location.isValid();
        data.status.bits.north_south = data.latitude < 0;
        data.status.bits.east_west = data.longitude < 0;
        
        // Time (unchanged)
        data.time.hours = tiny_gps_.time.hour();
        data.time.minutes = tiny_gps_.time.minute();
        data.time.seconds = tiny_gps_.time.second();
        data.time.milliseconds = tiny_gps_.time.centisecond() * 10;

        ESP_LOGI(TAG, "-----------------------------------");
        ESP_LOGI(TAG, "latitude: %" PRIu32, data.latitude);
        ESP_LOGI(TAG, "longitude: %" PRIu32, data.longitude);
        ESP_LOGI(TAG, "fix: %" PRIu8, data.quality.fix_type);
        ESP_LOGI(TAG, "satellites: %" PRIu8, data.quality.satellites);
        ESP_LOGI(TAG, "-----------------------------------");

        // Update vehicle data pool
        VehicleData::instance().updateGPS(data);
    }
}

}