#include "sbus.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "SBUS";

SBUS::SBUS(uart_port_t uart_num) : uart_num(uart_num) {
    parser_state = 0;
    current_byte = 0;
    previous_byte = SBUS_FOOTER;
    new_packet_available = false;
    ring_buffer.head = 0;
    ring_buffer.tail = 0;
    is_initialized = false;
}

SBUS::~SBUS() {
    if (sbus_task_handle) {
        vTaskDelete(sbus_task_handle);
    }
    if (uart_queue) {
        uart_driver_delete(uart_num);
    }
}

esp_err_t SBUS::begin(gpio_num_t rx_pin, gpio_num_t tx_pin, bool inverted,
                      uint32_t baud_rate) {
    esp_err_t ret = configureUART(rx_pin, tx_pin, inverted, baud_rate);
    if (ret != ESP_OK) {
        return ret;
    }
    
    BaseType_t task_created = xTaskCreate(
        sbusTask,
        "sbus_task",
        4096,
        this,
        5,  // Priority
        &sbus_task_handle
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SBUS task");
        return ESP_FAIL;
    }

    if (task_created == pdPASS) {
        is_initialized = true;
    }
    
    return ESP_OK;
}

void SBUS::sbusTask(void* parameters) {
    SBUS* sbus = static_cast<SBUS*>(parameters);
    uart_event_t event;
    uint8_t rx_buffer[256];
    
    while (true) {
        if (xQueueReceive(sbus->uart_queue, &event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                size_t length;
                ESP_ERROR_CHECK(uart_get_buffered_data_len(sbus->uart_num, &length));
                
                if (length > 0) {
                    length = uart_read_bytes(sbus->uart_num, rx_buffer, length, pdMS_TO_TICKS(1));
                    
                    for (size_t i = 0; i < length; i++) {
                        // Simple ring buffer - always write and advance
                        sbus->ring_buffer.buffer[sbus->ring_buffer.head] = rx_buffer[i];
                        sbus->ring_buffer.head = (sbus->ring_buffer.head + 1) % RING_BUFFER_SIZE;
                        
                        // Move tail if we're about to overflow
                        if (sbus->ring_buffer.head == sbus->ring_buffer.tail) {
                            sbus->ring_buffer.tail = (sbus->ring_buffer.tail + 1) % RING_BUFFER_SIZE;
                        }
                    }
                    
                    sbus->processBytes();
                }
            }
        }
    }
}

bool SBUS::processBytes() {
    while (ring_buffer.tail != ring_buffer.head) {
        current_byte = ring_buffer.buffer[ring_buffer.tail];
        ring_buffer.tail = (ring_buffer.tail + 1) % RING_BUFFER_SIZE;
        
        switch (parser_state) {
            case 0: // Looking for header
                if (current_byte == SBUS_HEADER && previous_byte == SBUS_FOOTER) {
                    parser_state++;
                }
                break;
                
            case 1 ... 23: // Collecting payload
                payload[parser_state - 1] = current_byte;
                parser_state++;
                break;
                
            case 24: // Footer
                if (current_byte == SBUS_FOOTER) {
                    parsePacket();
                    new_packet_available = true;
                }
                parser_state = 0;
                break;
        }
        
        previous_byte = current_byte;
    }
    return new_packet_available;
}

void SBUS::parsePacket() {
    // Decode 11-bit channel data
    latest_packet.channels[0]  = ((payload[0]    | payload[1] <<8)                     & 0x07FF);
    latest_packet.channels[1]  = ((payload[1]>>3 | payload[2] <<5)                     & 0x07FF);
    latest_packet.channels[2]  = ((payload[2]>>6 | payload[3] <<2 | payload[4]<<10)    & 0x07FF);
    latest_packet.channels[3]  = ((payload[4]>>1 | payload[5] <<7)                     & 0x07FF);
    latest_packet.channels[4]  = ((payload[5]>>4 | payload[6] <<4)                     & 0x07FF);
    latest_packet.channels[5]  = ((payload[6]>>7 | payload[7] <<1 | payload[8]<<9)     & 0x07FF);
    latest_packet.channels[6]  = ((payload[8]>>2 | payload[9] <<6)                     & 0x07FF);
    latest_packet.channels[7]  = ((payload[9]>>5 | payload[10]<<3)                     & 0x07FF);
    latest_packet.channels[8]  = ((payload[11]   | payload[12]<<8)                     & 0x07FF);
    latest_packet.channels[9]  = ((payload[12]>>3| payload[13]<<5)                     & 0x07FF);
    latest_packet.channels[10] = ((payload[13]>>6| payload[14]<<2 | payload[15]<<10)   & 0x07FF);
    latest_packet.channels[11] = ((payload[15]>>1| payload[16]<<7)                     & 0x07FF);
    latest_packet.channels[12] = ((payload[16]>>4| payload[17]<<4)                     & 0x07FF);
    latest_packet.channels[13] = ((payload[17]>>7| payload[18]<<1 | payload[19]<<9)    & 0x07FF);
    latest_packet.channels[14] = ((payload[19]>>2| payload[20]<<6)                     & 0x07FF);
    latest_packet.channels[15] = ((payload[20]>>5| payload[21]<<3)                     & 0x07FF);
}

esp_err_t SBUS::configureUART(gpio_num_t rx_pin, gpio_num_t tx_pin, 
                             bool inverted, uint32_t baud_rate) {
    uart_config_t uart_config = {
        .baud_rate = static_cast<int>(baud_rate),
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB
    };
    
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, 
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    if (inverted) {
        ESP_ERROR_CHECK(uart_set_line_inverse(uart_num, 
                       UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV));
    }
    
    const int uart_buffer_size = 256;
    const int uart_queue_size = 20;
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size * 2, 
                                      uart_buffer_size * 2, uart_queue_size, 
                                      &uart_queue, 0));
    
    return ESP_OK;
}

bool SBUS::read(uint16_t* channels) {
    if (!new_packet_available) {
        return false;
    }
    
    if (channels) {
        memcpy(channels, latest_packet.channels, sizeof(latest_packet.channels));
    }
    
    new_packet_available = false;
    return true;
}