#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "freertos/FreeRTOS.h"

#include "blackboard/topic_ids.h"

namespace blackboard {

// Must be called once before any publish(). Creates the global log ring.
void init();

// Record header layout, see docs/blackboard.md:
//   [0]    topic_id
//   [1]    payload_len
//   [2..7] timestamp_us (48-bit little-endian)
//   [8..]  payload
static constexpr size_t RECORD_HEADER_SIZE = 8;

// Internal ring plumbing. ring_append is called by Channel<T>::publish; the
// receive/release pair is the logger's drain API. Producers never block here:
// a full ring drops the record and bumps dropped_records().
void ring_append(uint8_t topic_id, const void* payload, size_t len);
const void* ring_receive(size_t* out_len, TickType_t timeout_ticks);
void ring_release(const void* item);
uint32_t dropped_records();

// Seqlock latest-value snapshot: single writer (the producer), many readers,
// readers copy and never block. A reader that races a write retries.
template <typename T>
class Snapshot {
    static_assert(std::is_trivially_copyable_v<T>,
                  "blackboard topic types must be trivially copyable");

    std::atomic<uint32_t> seq_{0};
    T value_{};

public:
    void store(const T& v) {
        uint32_t s = seq_.load(std::memory_order_relaxed) + 1;
        seq_.store(s, std::memory_order_relaxed);              // odd: write in progress
        std::atomic_thread_fence(std::memory_order_release);
        value_ = v;
        std::atomic_thread_fence(std::memory_order_release);
        seq_.store(s + 1, std::memory_order_relaxed);          // even: done
    }

    T load() const {
        T out;
        uint32_t s0, s1;
        do {
            s0 = seq_.load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            out = value_;
            std::atomic_thread_fence(std::memory_order_acquire);
            s1 = seq_.load(std::memory_order_relaxed);
        } while ((s0 & 1u) || s0 != s1);
        return out;
    }
};

// Bind a topic type to its id by specialising this in the owning component's
// header. Left undefined on purpose: publishing an unbound type is a compile
// error, so an id can never be forgotten.
template <typename T>
struct Topic;

template <typename T>
class Channel {
    Snapshot<T> snap_;

public:
    void publish(const T& v) {
        snap_.store(v);
        ring_append(static_cast<uint8_t>(Topic<T>::id), &v, sizeof(T));
    }
    T latest() const { return snap_.load(); }
};

template <typename T>
Channel<T>& channel_for() {
    static Channel<T> ch;   // lazy, thread-safe, one instance per type
    return ch;
}

template <typename T>
void publish(const T& v) {
    channel_for<T>().publish(v);
}

template <typename T>
T latest() {
    return channel_for<T>().latest();
}

}
