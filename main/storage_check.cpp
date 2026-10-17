#include "storage_check.hpp"
#include "sdkconfig.h"

#if CONFIG_CAR_SD_READ_WRITE_TEST
#include <algorithm>
#include <atomic>
#include <cstdio>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "imu.h"
#include "storage/card.hpp"
#include "storage/read_write_test.hpp"

namespace {
constexpr const char* tag = "sd_check";

struct ImuObservation {
    uint64_t maximum_age_us = 0;
    bool unavailable = false;
    unsigned observed_updates = 0;
    uint64_t last_timestamp = 0;
    uint64_t maximum_update_gap_us = 0;
    unsigned gaps_over_25ms = 0;
    unsigned gaps_over_50ms = 0;
    unsigned gaps_over_100ms = 0;

    void sample() {
        const auto gyro = imu::IMU::instance().samples().gyro;
        const auto now = uint64_t(esp_timer_get_time());
        if (!gyro.valid || gyro.timestamp_us > now) {
            unavailable = true;
            return;
        }
        if (gyro.timestamp_us != last_timestamp) {
            ++observed_updates;
            if (last_timestamp && gyro.timestamp_us > last_timestamp) {
                const auto gap = gyro.timestamp_us - last_timestamp;
                maximum_update_gap_us = std::max(maximum_update_gap_us, gap);
                gaps_over_25ms += gap > 25000;
                gaps_over_50ms += gap > 50000;
                gaps_over_100ms += gap > 100000;
            }
        }
        last_timestamp = gyro.timestamp_us;
        maximum_age_us = std::max(maximum_age_us, now - gyro.timestamp_us);
    }
};

struct ImuObserver {
    ImuObservation observation;
    std::atomic<bool> stop{false};
    SemaphoreHandle_t done = nullptr;
};

void observe_during_transfers(void* context) {
    auto& observer = *static_cast<ImuObserver*>(context);
    do {
        observer.observation.sample();
        vTaskDelay(1);
    } while (!observer.stop.load());
    observer.observation.sample();
    xSemaphoreGive(observer.done);
    vTaskDelete(nullptr);
}

void yield_between_blocks(void*) {
    vTaskDelay(1);
}

double kib_per_second(std::size_t bytes, uint64_t duration_us) {
    return duration_us ? double(bytes) * 1000000.0 / (1024.0 * double(duration_us)) : 0.0;
}
}

void check_sd_read_write() {
    ImuObservation idle_imu;
    for (unsigned i = 0; i < 100; ++i) {
        idle_imu.sample();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(tag, "Idle IMU observed_updates=%u max_gyro_age=%" PRIu64 "us unavailable=%d",
              idle_imu.observed_updates, idle_imu.maximum_age_us, idle_imu.unavailable);

    char path[64];
    std::snprintf(path, sizeof(path), "%s/T%07" PRIX32 ".BIN",
                  storage::Card::mount_path, esp_random() & 0x0fffffffU);
    ESP_LOGI(tag, "Checking %u KiB in %s", CONFIG_CAR_SD_TEST_SIZE_KIB, path);
    ImuObserver observer;
    observer.done = xSemaphoreCreateBinary();
    if (!observer.done) { ESP_LOGE(tag, "Not enough RAM for IMU observer"); return; }
    if (xTaskCreatePinnedToCore(observe_during_transfers, "sd_check_imu", 4096,
                                &observer, 5, nullptr, 1) != pdPASS) {
        vSemaphoreDelete(observer.done);
        ESP_LOGE(tag, "Cannot start IMU observer");
        return;
    }
    const auto test_start = esp_timer_get_time();
    storage::ReadWriteResult result;
    const auto error = storage::test_read_write(path, std::size_t(CONFIG_CAR_SD_TEST_SIZE_KIB) * 1024,
                                               result, yield_between_blocks, nullptr,
                                               CONFIG_CAR_SD_TEST_TRANSFER_BYTES);
    observer.stop.store(true);
    xSemaphoreTake(observer.done, portMAX_DELAY);
    vSemaphoreDelete(observer.done);
    const auto& imu = observer.observation;
    const auto duration_us = esp_timer_get_time() - test_start;
    if (error != ESP_OK) {
        ESP_LOGE(tag, "Read/write failed: %s errno=%d wrote=%zu read=%zu mismatch=%zu file=%s",
                  esp_err_to_name(error), result.filesystem_errno, result.bytes_written,
                  result.bytes_read, result.mismatch_offset, path);
    } else {
        ESP_LOGI(tag, "Verified %zu B write=%.0f KiB/s read=%.0f KiB/s max_write=%" PRIu64
                      "us max_read=%" PRIu64 "us max_sync=%" PRIu64 "us",
                  result.bytes_read, kib_per_second(result.bytes_written, result.write_elapsed_us),
                  kib_per_second(result.bytes_read, result.read_elapsed_us), result.longest_write_us,
                  result.longest_read_us, result.longest_sync_us);
    }
    ESP_LOGI(tag, "IMU duration=%" PRId64 "us updates=%u max_age=%" PRIu64
                  "us max_update_gap=%" PRIu64 "us gaps>25ms=%u >50ms=%u >100ms=%u unavailable=%d",
              duration_us, imu.observed_updates, imu.maximum_age_us, imu.maximum_update_gap_us,
              imu.gaps_over_25ms, imu.gaps_over_50ms, imu.gaps_over_100ms, imu.unavailable);
    ImuObservation recovered_imu;
    for (unsigned i = 0; i < 100; ++i) {
        recovered_imu.sample();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(tag, "After SD IMU observed_updates=%u max_gyro_age=%" PRIu64 "us unavailable=%d",
              recovered_imu.observed_updates, recovered_imu.maximum_age_us, recovered_imu.unavailable);
}
#else
void check_sd_read_write() {}
#endif
