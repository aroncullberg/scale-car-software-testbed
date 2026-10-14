#include "blackboard/blackboard.h"

#include <cstring>

#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace blackboard {

static const char* TAG = "blackboard";

static constexpr size_t RING_SIZE_BYTES = 16 * 1024;

static RingbufHandle_t s_ring = nullptr;
static std::atomic<uint32_t> s_dropped{0};

void init() {
    if (s_ring) {
        return;
    }
    s_ring = xRingbufferCreate(RING_SIZE_BYTES, RINGBUF_TYPE_NOSPLIT);
    if (!s_ring) {
        ESP_LOGE(TAG, "failed to create %u byte log ring", (unsigned)RING_SIZE_BYTES);
        return;
    }
    ESP_LOGI(TAG, "initialized, %u byte log ring", (unsigned)RING_SIZE_BYTES);
}

void ring_append(uint8_t topic_id, const void* payload, size_t len) {
    if (!s_ring || len > 255) {
        s_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    const size_t total = RECORD_HEADER_SIZE + len;
    void* slot = nullptr;
    // 0-tick acquire: a publisher (including the control loop) never blocks on
    // logging. If the ring is full the record is dropped and counted.
    if (xRingbufferSendAcquire(s_ring, &slot, total, 0) != pdTRUE) {
        s_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    uint64_t ts = static_cast<uint64_t>(esp_timer_get_time());
    uint8_t* p = static_cast<uint8_t*>(slot);
    p[0] = topic_id;
    p[1] = static_cast<uint8_t>(len);
    p[2] = static_cast<uint8_t>(ts & 0xff);
    p[3] = static_cast<uint8_t>((ts >> 8) & 0xff);
    p[4] = static_cast<uint8_t>((ts >> 16) & 0xff);
    p[5] = static_cast<uint8_t>((ts >> 24) & 0xff);
    p[6] = static_cast<uint8_t>((ts >> 32) & 0xff);
    p[7] = static_cast<uint8_t>((ts >> 40) & 0xff);
    memcpy(p + RECORD_HEADER_SIZE, payload, len);

    xRingbufferSendComplete(s_ring, slot);
}

const void* ring_receive(size_t* out_len, TickType_t timeout_ticks) {
    if (!s_ring) {
        if (out_len) {
            *out_len = 0;
        }
        return nullptr;
    }
    size_t len = 0;
    void* item = xRingbufferReceive(s_ring, &len, timeout_ticks);
    if (out_len) {
        *out_len = len;
    }
    return item;
}

void ring_release(const void* item) {
    if (s_ring && item) {
        vRingbufferReturnItem(s_ring, const_cast<void*>(item));
    }
}

uint32_t dropped_records() {
    return s_dropped.load(std::memory_order_relaxed);
}

}
