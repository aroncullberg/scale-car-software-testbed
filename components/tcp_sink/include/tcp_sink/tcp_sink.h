#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "logger/sink.h"

namespace tcp_sink {

struct Config {
    const char* ssid = "car-telemetry";
    const char* password = "carlogger123"; // >= 8 chars for WPA2, "" for open
    uint16_t port = 3333;
    uint8_t channel = 1;
    uint8_t max_clients = 1;
    // Store-and-forward ring, allocated in PSRAM. Holds records while no client
    // is connected or while out of range; wraps overwrite-oldest when full.
    size_t buffer_bytes = 4 * 1024 * 1024;
};

// logger::Sink that streams records to a laptop over WiFi.
//
// Brings up a SoftAP (the car's own network) and a TCP server. write() runs on
// the logger task and only copies the record into the PSRAM ring, never
// blocking on the network. A separate task drains the ring to the connected
// client.
class TcpSink : public logger::Sink {
public:
    esp_err_t start(const Config& config);

    bool write(const uint8_t* record, size_t len) override;

    uint32_t dropped_records() const { return dropped_.load(std::memory_order_relaxed); }

private:
    static void server_task_entry(void* arg);
    void server_task();
    esp_err_t wifi_init_softap();

    // Record-granular ring over a byte buffer. Each entry is a 2-byte length
    // prefix (internal, not sent) followed by the record bytes.
    bool ring_push(const uint8_t* rec, size_t len);   // drops oldest to make room
    size_t ring_pop(uint8_t* out, size_t max);        // 0 if empty

    void buf_write(const uint8_t* src, size_t n);
    void buf_read(size_t from_tail, uint8_t* dst, size_t n) const;

    Config cfg_{};
    uint8_t* buf_{nullptr};
    size_t cap_{0};
    size_t head_{0};
    size_t tail_{0};
    size_t used_{0};
    SemaphoreHandle_t mtx_{nullptr};
    TaskHandle_t task_{nullptr};
    std::atomic<uint32_t> dropped_{0};
};

}
