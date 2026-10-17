#include "passthrough_usb.hpp"

#include <cstring>
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

namespace vehicle::detail {
namespace {
bool usb_output(void*, const uint8_t* data, size_t size) {
    size_t sent = 0;
    while (sent < size) {
        const int count = usb_serial_jtag_write_bytes(data + sent, size - sent, pdMS_TO_TICKS(100));
        if (count <= 0) return false;
        sent += count;
    }
    return usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100)) == ESP_OK;
}
} // namespace

esp_err_t start_passthrough(const std::array<MotorConfig, motor_count>& motors,
                            esc_passthrough_lifecycle_t lifecycle,
                            esc_passthrough_t** session) {
    usb_serial_jtag_driver_config_t usb = {.tx_buffer_size = 2048, .rx_buffer_size = 2048};
    auto error = usb_serial_jtag_driver_install(&usb);
    if (error != ESP_OK) return error;
    usb_serial_jtag_vfs_use_driver();
    auto config = esc_passthrough_config_default();
    for (size_t i = 0; i < motors.size(); ++i) config.pins[i] = motors[i].gpio;
    config.uart_port = 0; // UART1 belongs to RC and UART2 to GPS.
    config.output = usb_output;
    config.lifecycle = lifecycle;
    error = esp_read_mac(config.uid, ESP_MAC_BASE);
    if (error == ESP_OK) {
        std::memcpy(config.uid + 6, "CAR026", 6);
        std::memcpy(config.board, "CAR3", 4);
        error = esc_passthrough_create(&config, session);
    }
    if (error != ESP_OK) {
        usb_serial_jtag_vfs_use_nonblocking();
        usb_serial_jtag_driver_uninstall();
    }
    return error;
}

esp_err_t stop_passthrough(esc_passthrough_t** session) {
    if (!*session) return ESP_OK;
    const auto error = esc_passthrough_destroy(*session);
    if (error != ESP_OK) return error;
    *session = nullptr;
    usb_serial_jtag_vfs_use_nonblocking();
    return usb_serial_jtag_driver_uninstall();
}

void pump_passthrough(esc_passthrough_t* session, bool& was_connected) {
    const bool connected = usb_serial_jtag_is_connected();
    if (was_connected && !connected) esc_passthrough_transport_lost(session);
    was_connected = connected;
    if (connected) {
        uint8_t input[512];
        const int size = usb_serial_jtag_read_bytes(input, sizeof(input), 0);
        if (size > 0) esc_passthrough_feed(session, input, size,
            static_cast<uint32_t>(esp_timer_get_time() / 1000));
    }
    esc_passthrough_poll(session, static_cast<uint32_t>(esp_timer_get_time() / 1000));
}

} // namespace vehicle::detail
