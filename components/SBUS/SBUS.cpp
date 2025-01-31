#include "SBUS.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "data_pool.h"

static const char* TAG = "SBUS";

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
    return configureUART();
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
        CONFIG_SBUS_TASK_STACK_SIZE,        // stack size (Kconfig)
        this,                               // ???
        CONFIG_SBUS_TASK_PRIORITY,          // Task priority (Kconfig)
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
    // SBUS channel data is packed across 22 bytes (11 bits/ch)
    uint16_t channels[16] = {0};
    int byte_index = 1; //skip start byte
    int bit_index = 0;

    for (int ch = 0; ch < 16; ch++) {
        channels[ch] = 0;
        for (int bit = 0; bit < 11; bit++) {
            if (frame[byte_index] & (1 << bit_index)) {
                channels[ch] |= (1 << bit);
            }
            
            bit_index++;
            if (bit_index == 8) {
                bit_index = 0;
                byte_index++;
            }
        }
    }

    updateChannelValues(channels);
}


void SBUS::updateChannelValues(const uint16_t* raw_channels) {
        // SBUS values are 11-bit (0-2047)
    static const SbusChannelConfig channel_configs[] = {
        {
            .type =
            #if defined(CONFIG_SBUS_CH0_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH0_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH0_MIN,
            // .center_raw = CONFIG_SBUS_CH0_CENTER,
            .max_raw = CONFIG_SBUS_CH0_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH1_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH1_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH1_MIN,
            .center_raw = CONFIG_SBUS_CH1_CENTER,
            .max_raw = CONFIG_SBUS_CH1_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH2_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH2_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH2_MIN,
            .center_raw = CONFIG_SBUS_CH2_CENTER,
            .max_raw = CONFIG_SBUS_CH2_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH3_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH3_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH3_MIN,
            .center_raw = CONFIG_SBUS_CH3_CENTER,
            .max_raw = CONFIG_SBUS_CH3_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH4_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH4_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH4_MIN,
            .center_raw = CONFIG_SBUS_CH4_CENTER,
            .max_raw = CONFIG_SBUS_CH4_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH5_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH5_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH5_MIN,
            .center_raw = CONFIG_SBUS_CH5_CENTER,
            .max_raw = CONFIG_SBUS_CH5_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH6_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH6_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH6_MIN,
            .center_raw = CONFIG_SBUS_CH6_CENTER,
            .max_raw = CONFIG_SBUS_CH6_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH7_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH7_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH7_MIN,
            .center_raw = CONFIG_SBUS_CH7_CENTER,
            .max_raw = CONFIG_SBUS_CH7_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH8_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH8_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH8_MIN,
            .center_raw = CONFIG_SBUS_CH8_CENTER,
            .max_raw = CONFIG_SBUS_CH8_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH9_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH9_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH9_MIN,
            .center_raw = CONFIG_SBUS_CH9_CENTER,
            .max_raw = CONFIG_SBUS_CH9_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH10_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH10_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH10_MIN,
            .center_raw = CONFIG_SBUS_CH10_CENTER,
            .max_raw = CONFIG_SBUS_CH10_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH11_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH11_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH11_MIN,
            .center_raw = CONFIG_SBUS_CH11_CENTER,
            .max_raw = CONFIG_SBUS_CH11_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH12_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH12_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH12_MIN,
            .center_raw = CONFIG_SBUS_CH12_CENTER,
            .max_raw = CONFIG_SBUS_CH12_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH13_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH13_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH13_MIN,
            .center_raw = CONFIG_SBUS_CH13_CENTER,
            .max_raw = CONFIG_SBUS_CH13_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH14_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH14_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH14_MIN,
            .center_raw = CONFIG_SBUS_CH14_CENTER,
            .max_raw = CONFIG_SBUS_CH14_MAX
        },
        {
            .type =
            #if defined(CONFIG_SBUS_CH15_SYMMETRIC)
                SbusChannelType::SYMMETRIC
            #elif defined(CONFIG_SBUS_CH15_UNIPOLAR)
                SbusChannelType::UNIPOLAR
            #else
                SbusChannelType::BINARY
            #endif
            ,
            .min_raw = CONFIG_SBUS_CH15_MIN,
            .center_raw = CONFIG_SBUS_CH15_CENTER,
            .max_raw = CONFIG_SBUS_CH15_MAX
        },
    };

    // scale each channel according to config
    for (int i = 0; i < 16; i++) {
        current_data_.channels[i] = scaleChannelValue(raw_channels[i], channel_configs[i]);
    }
    
    // Update signal quality
    monitorSignalQuality();

    VehicleData::instance().updateSBUS(current_data_);
}

float SBUS::scaleChannelValue(uint16_t raw_value, const SbusChannelConfig& config) {
    float scaled_value = 0.0f;

        switch (config.type) {
        case SbusChannelType::SYMMETRIC:
            if (raw_value < config.center_raw) {
                // scale from min to center -1 -> 0
                scaled_value = -1.0f +
                    (float)(raw_value - config.min_raw) /
                    (float)(config.center_raw - config.min_raw);
            } else {
                // scale center to max 0 -> 1
                scaled_value = 
                    (float)(raw_value - config.center_raw) /
                    (float)(config.max_raw - config.center_raw);
            }
            break;
        case SbusChannelType::UNIPOLAR:
            // scale from min to max (0 to 1)
            scaled_value = 
                (float)(raw_value - config.min_raw) /
                (float)(config.max_raw - config.min_raw);
            break;
        case SbusChannelType::BINARY:
            // threshold at midpoint between min and max
            uint16_t treshhold = (config.min_raw + config.max_raw) / 2;
            scaled_value = (raw_value >= treshhold) ? 1.0f : 0.0f;
            break;
    }

    // clamp values
    if (scaled_value < -1.0f) scaled_value = -1.0f;
    if (scaled_value > 1.0f) scaled_value = 1.0f;

    return scaled_value;

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

const char* SBUS::getChannelName(SbusChannel channel) {
    static const char* channel_names[] = {
        "Throttle",
        "Steering",
        "AUX1",
        "AUX2",
        "AUX3",
        "AUX4",
        "AUX5",
        "AUX6",
        "AUX7",
        "AUX8",
        "AUX9",
        "AUX10",
        "AUX11",
        "AUX12",
        "AUX13",
        "AUX14"
    };

    if (static_cast<uint8_t>(channel) >= static_cast<uint8_t>(SbusChannel::CHANNEL_COUNT)) {
        return "Invalid";
    }

    return channel_names[static_cast<uint8_t>(channel)];
}


}