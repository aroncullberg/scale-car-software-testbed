#include <cstring>
#include <algorithm>
#include "gps.h"
#include "esp_log.h"
#include <data_pool.h>
#include <cinttypes>
#include "config_manager.h"

namespace sensor
{
    GPS::GPS(const Config &config) {
        config_ = config;
        ESP_LOGI(TAG, "GPS instance created");

        callback_ = [this] { this->updateFromConfig(); };
        ConfigManager::instance().registerCallback(callback_);
    }

    GPS::~GPS() {
        stop();
        ESP_LOGI(TAG, "GPS instance destroyed");
    }

    esp_err_t GPS::init() {
        // ESP_LOGI(TAG, "initializing GPS on UART%d (TX:%d, RX:%d, baud:%d)",
        //  config_.uart_num, config_.uart_tx_pin, config_.uart_rx_pin, config_.baud_rate);
        // In your init() function, add:
        ESP_LOGI(TAG, "GPS Module Info:");
        ESP_LOGI(TAG, "  UART Num: %d", config_.uart_num);
        ESP_LOGI(TAG, "  RX Pin: %d", config_.uart_rx_pin);
        ESP_LOGI(TAG, "  Baud Rate: %d", config_.baud_rate);

        updateFromConfig();

        return configureUART();
    }

    void GPS::updateFromConfig() {
        bool new_debug_logging = ConfigManager::instance().getBool("gps/logging", debug_logging_);
        if (new_debug_logging != debug_logging_) {
            ESP_LOGI(TAG, "Debug logging changed: %s -> %s",
                     debug_logging_ ? "true" : "false",
                     new_debug_logging ? "true" : "false");
            debug_logging_ = new_debug_logging;
        }
        bool new_verbose_logging = ConfigManager::instance().getBool("gps/verbose", verbose_logging_);
        if (new_verbose_logging != verbose_logging_) {
            ESP_LOGI(TAG, "Verbose logging changed: %s -> %s",
                     verbose_logging_ ? "true" : "false",
                     new_verbose_logging ? "true" : "false");
            verbose_logging_ = new_verbose_logging;
        }
    }

    esp_err_t GPS::configureUART() {
        uart_config_t uart_config = {
            .baud_rate = config_.baud_rate, // Explicitly set to 9600
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB, // Try explicit clock source
        };

        esp_err_t err = uart_param_config(config_.uart_num, &uart_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "UART param config failed (%d)", err);
            return err;
        }

        // Set pins with explicit pull settings
        err = uart_set_pin(config_.uart_num,
                           config_.uart_tx_pin,
                           config_.uart_rx_pin,
                           GPIO_NUM_NC, // No RTS
                           GPIO_NUM_NC); // No CTS
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "UART set pin failed (%d)", err);
            return err;
        }

        // Install driver with larger buffers
        err = uart_driver_install(config_.uart_num,
                                  config_.rx_buffer_size, // Larger RX buffer
                                  config_.tx_buffer_size, // No TX buffer
                                  0, // No event queue
                                  nullptr, // No queue handle
                                  0); // No interrupt flags
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "UART driver install failed (%d)", err);
            return err;
        }

        // Send UBX command to disable UBX protocol
        const auto ubx_cfg_prt =
                "\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x00\xC2\x01\x00\x07\x00\x01\x00\x00\x00\x00\x00\xC0\x7E";
        uart_write_bytes(config_.uart_num, ubx_cfg_prt, 28); // Exact 28-byte UBX packet

        vTaskDelay(pdMS_TO_TICKS(100));

        const uint8_t ubx_cfg_rate[] = {
            0xB5, 0x62, // Header
            0x06, 0x08, // Class/ID (CFG-RATE)
            0x06, 0x00, // Length
            0xC8, 0x00, // measRate (200ms = 5Hz)
            0x01, 0x00, // navRate (1)
            0x01, 0x00, // timeRef (1 = GPS time)
            0xDE, 0x6A // Checksum
        };
        int written = uart_write_bytes(config_.uart_num, ubx_cfg_rate, sizeof(ubx_cfg_rate));
        if (written != sizeof(ubx_cfg_rate)) {
            ESP_LOGE(TAG, "Failed to send UBX-CFG-RATE command");
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(100));


        return ESP_OK;
    }

    esp_err_t GPS::start() {
        if (is_running) {
            ESP_LOGW(TAG, "GPS already active");
            return ESP_ERR_INVALID_STATE;
        }

        BaseType_t task_created = xTaskCreatePinnedToCore(
            gpsTask, // task function
            "gps_task", // task name
            4096, // stack size (Kconfig)
            this, // ???
            5, // Task priority (Kconfig)
            &task_handle_, // self explanatory
            1
        );

        BaseType_t task_created1 = xTaskCreatePinnedToCore(
            reportingTask, // task function
            "gps_report", // task name
            4096, // stack size (smaller than main GPS task)
            this, // parameter
            3, // lower priority than main GPS task
            &reporting_task_handle_, // task handle
            1 // core
        );

        if (task_created1 != pdPASS) {
            ESP_LOGE(TAG, "Failed to create GPS task");
            return ESP_ERR_NO_MEM;
        }

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

        if (reporting_task_handle_ != nullptr) {
            vTaskDelete(reporting_task_handle_);
            reporting_task_handle_ = nullptr;
        }


        is_running = false;
        ESP_LOGI(TAG, "GPS stopped");
        return ESP_OK;
    }

    void GPS::reportingTask(void *parameters) {
        const auto gps = static_cast<GPS *>(parameters);
        TickType_t last_wake_time = xTaskGetTickCount();

        ESP_LOGI(TAG, "GPS reporting task started");

        while (true) {
            if (!gps->debug_logging_) {
                vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(gps->config_.targetFreq));
                continue;
            }
            GpsData current_data = VehicleData::instance().getGPS();

            if (!current_data.status.bits.valid_fix) {
                ESP_LOGW(TAG, "No valid GPS fix (satellites: %u)", current_data.quality.satellites);
                vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(gps->config_.targetFreq));
                continue;
            }

            uint32_t max_speed = 0;
            max_speed = gps->max_speed_mmps_;

            // Convert speeds from mm/s to km/h for display
            float current_speed_kmh = current_data.speed_mmps / 1000.0f * 3.6f;
            float max_speed_kmh = max_speed / 1000.0f * 3.6f;

            ESP_LOGI(TAG, "%3.2f kmh (%3.2f) | sat count: %d", current_speed_kmh, max_speed_kmh,
                     current_data.quality.satellites);

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(gps->config_.targetFreq));
        }
    }


    void GPS::gpsTask(void *parameters) {
        const auto gps = static_cast<GPS *>(parameters);
        const uart_port_t uart_num = gps->config_.uart_num;

        // Buffer for reading UART data

        ESP_LOGI(TAG, "GPS task started");
        TickType_t last_wake_time = xTaskGetTickCount();

        while (true) {
            // Read available UART data
            size_t buffered_length;
            uart_get_buffered_data_len(uart_num, &buffered_length);

            if (buffered_length > 0) {
                uint8_t data[512];
                int read_length = uart_read_bytes(uart_num,
                                                  data,
                                                  std::min(buffered_length,
                                                           sizeof(data)),
                                                  0); // No waiting

                if (read_length > 0) {
                    if (gps->verbose_logging_) {
                        // ESP_LOGI(TAG, "Read %d bytes from UART", read_length);
                        ESP_LOGI(TAG, "%s", data);
                    }
                    for (int i = 0; i < read_length; i++) {
                        gps->tiny_gps_.encode(data[i]);
                    }
                    gps->processGPSData();
                }
            }

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(gps->config_.targetFreq));
            if (gps->log_freq_) { // TODO: connect log_freq_ to configmanager
                static TickType_t prev_wake = 0;
                TickType_t now = xTaskGetTickCount();
                if (prev_wake != 0) {
                    TickType_t delta_ticks = now - prev_wake;
                    if (delta_ticks > 0) {
                        uint32_t freq_hz = 1000 / delta_ticks; // assuming ticks are in ms
                        ESP_LOGI(TAG, "Frequency: %lu Hz", freq_hz);
                    }
                }
                prev_wake = now;
            }
        }
    }


    void GPS::processGPSData() {
        if (tiny_gps_.location.isValid() && tiny_gps_.location.isUpdated()) {
            GpsData data;

            // Position data (unchanged)
            data.latitude = tiny_gps_.location.lat() * 10000000; // Convert to fixed point
            data.longitude = tiny_gps_.location.lng() * 10000000;
            data.altitude_mm = tiny_gps_.altitude.meters() * 1000;

            // Speed data (new)
            if (tiny_gps_.speed.isValid()) {
                // Convert from knots to m/s and store as mm/s for fixed point
                data.speed_mmps = tiny_gps_.speed.mps() * 1000;
                data.ground_course = tiny_gps_.course.deg() * 100;
                data.speed_valid = true;
            } else {
                data.speed_mmps = 0;
                data.ground_course = 0;
                data.speed_valid = false;
            }

            data.quality.fix_type = tiny_gps_.location.isValid() ? (tiny_gps_.altitude.isValid() ? 2 : 1) : 0;
            data.quality.satellites = tiny_gps_.satellites.value();
            data.quality.hdop = tiny_gps_.hdop.value() * 100; // Convert to fixed point

            data.status.bits.valid_fix = tiny_gps_.location.isValid();
            data.status.bits.north_south = data.latitude < 0;
            data.status.bits.east_west = data.longitude < 0;

            data.time.hours = tiny_gps_.time.hour();
            data.time.minutes = tiny_gps_.time.minute();
            data.time.seconds = tiny_gps_.time.second();
            data.time.milliseconds = tiny_gps_.time.centisecond() * 10;

            if (data.speed_valid && data.speed_mmps > 0) {
                max_speed_mmps_ = std::max(max_speed_mmps_, data.speed_mmps);
            }

            // Update vehicle data pool
            VehicleData::instance().updateGPS(data);
        }
    }
}
