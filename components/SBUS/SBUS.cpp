#include "SBUS.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "data_pool.h"


namespace sensor {

SBUS::SBUS(const Config& config) {
    config_t = config;
    ESP_LOGI(TAG, "SBUS instance created");
}

SBUS::~SBUS() {
    stop();
    ESP_LOGI(TAG, "SBUS instance destroyed");
}

esp_err_t SBUS::init() {
    ESP_LOGI(TAG, "initializing SBUS on UART%d (TX:%d, RX:%d)",
                    config_t.uart_num, config_t.uart_tx_pin, config_t.uart_rx_pin);

    ESP_ERROR_CHECK(configureUART());

    return ESP_OK;
}

esp_err_t SBUS::configureUART() {
    uart_config_t uart_config = {
        .baud_rate = config_t.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Configure UART parameters
    esp_err_t err = uart_param_config(config_t.uart_num, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART paramter configuration failed");
        return err;
    }

    // Set pins
    err = uart_set_pin(config_t.uart_num,
                        config_t.uart_tx_pin,
                        config_t.uart_rx_pin,
                        UART_PIN_NO_CHANGE, //No RTS
                        UART_PIN_NO_CHANGE); //No CTS
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART pin configuration failed");
        return err;
    }

    // Install uart driver
    // buffersizes need to be power of 2 (?)
    const int rx_buffer_size  = 512;
    const int tx_buffer_size = 0;   // We don't transmit

    err = uart_driver_install(config_t.uart_num,rx_buffer_size,tx_buffer_size,0,NULL,0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver installation failed");
        return err;
    }

    // enable singal inversion (sbus inverted yadayada)
    err = uart_set_line_inverse(config_t.uart_num, UART_SIGNAL_RXD_INV);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART signal inversion");
        return err;
    }

    return ESP_OK;
}

esp_err_t SBUS::start() {
    if (is_running) {
        ESP_LOGW(TAG, "SBUS already running");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t task_created = xTaskCreate(
        sbusTask,                           // task function
        "sbus_task",                        // task name
        4096,                               // stack
        this,                               // ???
        5,                                  // Task priority 
        &task_handle_                       // self explanatory 
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SBUS task");
        return ESP_ERR_NO_MEM;
    }

    is_running = true;
    ESP_LOGI(TAG, "SBUS Started");
    return ESP_OK;
}

esp_err_t SBUS::stop() {
    if (!is_running) {
        return ESP_OK;
    }

    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    is_running = false;
    ESP_LOGI(TAG, "SBUS stopped");
    return ESP_OK;
}

void SBUS::sbusTask(void* parameters) {
    SBUS* instance = static_cast<SBUS*>(parameters);
    const uint8_t start_byte = 0x0F;
    const uint8_t end_byte = 0x00;
    uint8_t byte;
    TickType_t last_frame_time = xTaskGetTickCount();

    uart_flush(instance->config_t.uart_num);

    while (true) {
        // Wait for start byte
        // NOTE: pdMS_TO_TICKS(5) take note and see if this time can be optimized. (boring math)
        while (uart_read_bytes(instance->config_t.uart_num, &byte, 1, pdMS_TO_TICKS(5)) > 0) { 
            if (byte == start_byte) {
                instance->frame_buffer_[0] = byte;
                instance->buffer_index_ = 1;
                break;
            }
        }

        if (instance->buffer_index_ == 1) {
            // Read the rest of the frame
            int remaining = FRAME_SIZE - 1;
            int read = uart_read_bytes(instance->config_t.uart_num, 
                                     &instance->frame_buffer_[1], 
                                     remaining, 
                                     pdMS_TO_TICKS(3));

            if (read == remaining && instance->frame_buffer_[FRAME_SIZE-1] == end_byte) {
                // Valid frame received
                instance->processFrame(instance->frame_buffer_, FRAME_SIZE);
                
                // Update timing
                TickType_t current_time = xTaskGetTickCount();
                float interval = (float)(current_time - last_frame_time) * portTICK_PERIOD_MS;
                if (interval > 0) {
                    instance->current_data_.quality.frame_interval_ms = interval;
                }
                last_frame_time = current_time;
            }
            
            instance->buffer_index_ = 0;
        }
        // vTaskDelay(1);
    }
}

void SBUS::processFrame(const uint8_t* frame, size_t len) {

    if (len != FRAME_SIZE) {
        ESP_LOGW(TAG, "invalid frame size %d", len);
        current_data_.quality.error_count++;
        return;
    }

    #if CONFIG_SBUS_LOG_RAW_FRAMES
        ESP_LOGI(TAG, "Frame: [%02X %02X %02X %02X %02X %02X %02X %02X ...]", 
                 frame[0], frame[1], frame[2], frame[3],
                 frame[4], frame[5], frame[6], frame[7]);
    #endif

    int byte_index = 1; // <- skip start byte
    int bit_index = 0;
    
    
    for (int ch = 0; ch < 16; ch++) {
        uint16_t raw_value = 0;

        // Extract the 11 bits for channel
        for (int bit = 0; bit < 11; bit++) {
            if (frame[byte_index] & (1 << bit_index)) {
                raw_value |= (1 << bit);
            }
            
            bit_index++;
            if (bit_index == 8) {
                bit_index = 0;
                byte_index++;
            }
        }

        current_data_.channels[ch] = scaleChannelValue(
            raw_value,
            CHANNEL_CONFIGS[ch][MIN],
            CHANNEL_CONFIGS[ch][MAX]
        );

        #if CONFIG_SBUS_DEBUG_LOGGING
            if ((ch == 0 || ch == 1) && xTaskGetTickCount() % 100 == 0) {
                ESP_LOGI(TAG, "CH%d: Raw=%4d, Scaled=%4d", 
                        ch, 
                        raw_value, 
                        current_data_.channels[ch]);
            }
        #endif
    }


    monitorSignalQuality();
    VehicleData::instance().updateSBUS(current_data_);
}


uint16_t SBUS::scaleChannelValue(uint16_t raw_value, uint16_t min_raw, uint16_t max_raw) {
    // if (raw_value < min_raw) raw_value = min_raw;
    // if (raw_value > max_raw) raw_value = max_raw;
    if (raw_value < min_raw || raw_value > max_raw * 1.1) {
        return 1000;
    } else {
        return 1000 + ((raw_value - min_raw) * 1000) / (max_raw - min_raw);
    }
    
    // Scale to 1000-2000 range
}

void SBUS::monitorSignalQuality() {
    static constexpr uint32_t FRAME_TIMEOUT_MS = 100; // singal lost after 100ms noshow
    static constexpr float NOMINAL_FRAME_INTERVAL = 14.0f; // SBUS runs at ~70Hz (14ms)
    static constexpr float FRAME_INTERVAL_TOLERANCE = 10.0f; // ±5ms

    // fraem ariving at expected rate
    bool timing_ok = abs(current_data_.quality.frame_interval_ms - NOMINAL_FRAME_INTERVAL) < FRAME_INTERVAL_TOLERANCE;

    // ESP_LOGI(TAG, "bool: %2d | %4f %4f", timing_ok, abs(current_data_.quality.frame_interval_ms - NOMINAL_FRAME_INTERVAL), FRAME_INTERVAL_TOLERANCE);

    /// update valid signal flag
    current_data_.quality.valid_signal = timing_ok;

    // calc loss percentage (rolling iwndow)
    static constexpr int WINDOW_SIZE = 100; // TODO: evlauate if this should be moved to menuconfig
    static int good_frames = 0;

    if (timing_ok) good_frames++;

    if (good_frames > WINDOW_SIZE) good_frames = WINDOW_SIZE;

    current_data_.quality.frame_loss_percent = (uint8_t)(100-(good_frames * 100 / WINDOW_SIZE));
}

// const char* SBUS::getChannelName(SbusChannel channel) {
//     static const char* channel_names[] = {
//         "Throttle",
//         "Steering",
//         "AUX1",
//         "AUX2",
//         "AUX3",
//         "AUX4",
//         "AUX5",
//         "AUX6",
//         "AUX7",
//         "AUX8",
//         "AUX9",
//         "AUX10",
//         "AUX11",
//         "AUX12",
//         "AUX13",
//         "AUX14"
//     };

//     if (static_cast<uint8_t>(channel) >= static_cast<uint8_t>(SbusChannel::CHANNEL_COUNT)) {
//         return "Invalid";
//     }

//     return channel_names[static_cast<uint8_t>(channel)];
// }


}