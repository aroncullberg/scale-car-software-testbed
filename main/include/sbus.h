#ifndef SBUS_H
#define SBUS_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

struct sbus_packet_t {
    uint16_t channels[16];
};

class SBUS {
public:
    SBUS(uart_port_t uart_num);
    ~SBUS();
    
    esp_err_t begin(gpio_num_t rx_pin, gpio_num_t tx_pin, 
                   bool inverted = false, 
                   uint32_t baud_rate = 100000);
    
    bool read(uint16_t* channels);

    bool is_ready() const { return is_initialized; }


private:
    static constexpr uint8_t SBUS_HEADER = 0x0F;
    static constexpr uint8_t SBUS_FOOTER = 0x00;
    static constexpr size_t RING_BUFFER_SIZE = 256;

    volatile bool is_initialized;

    
    TaskHandle_t sbus_task_handle;
    uart_port_t uart_num;
    QueueHandle_t uart_queue;
    
    // Latest packet data
    sbus_packet_t latest_packet;
    bool new_packet_available;
    
    // Ring buffer
    struct {
        uint8_t buffer[RING_BUFFER_SIZE];
        size_t head;
        size_t tail;
    } ring_buffer;
    
    // Processing state
    uint8_t parser_state;
    uint8_t current_byte;
    uint8_t previous_byte;
    uint8_t payload[24];
    
    static void sbusTask(void* parameters);
    esp_err_t configureUART(gpio_num_t rx_pin, gpio_num_t tx_pin, 
                           bool inverted, uint32_t baud_rate);
    bool processBytes();
    void parsePacket();
};

#endif // SBUS_H