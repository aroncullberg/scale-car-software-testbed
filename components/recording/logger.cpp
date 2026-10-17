#include "recording/logger.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <type_traits>
#include <unistd.h>
#include "esp_timer.h"
#include "sdkconfig.h"

namespace recording {
Logger::~Logger() { stop(); }

bool Logger::valid_filename(const char* name) {
    if (!name || std::strlen(name) != 12 || std::strcmp(name + 8, ".ULG")) return false;
    bool nonzero = false;
    for (int i = 0; i < 8; ++i) {
        if (name[i] < '0' || name[i] > '9') return false;
        nonzero |= name[i] != '0';
    }
    return nonzero;
}

esp_err_t Logger::start() {
    if (task_ || file_ >= 0) return ESP_ERR_INVALID_STATE;
    if (mkdir(directory, 0700) != 0 && errno != EEXIST) { fail(errno); return ESP_FAIL; }
    auto* dir = opendir(directory);
    if (!dir) { fail(errno); return ESP_FAIL; }
    uint32_t highest = 0;
    while (const auto* entry = readdir(dir)) {
        if (valid_filename(entry->d_name)) highest = std::max(highest, uint32_t(std::strtoul(entry->d_name, nullptr, 10)));
    }
    closedir(dir);
    if (highest >= 99999999) { fail(ENOSPC); return ESP_FAIL; }
    char name[13], path[64];
    std::snprintf(name, sizeof(name), "%08lu.ULG", (unsigned long)(highest + 1));
    std::snprintf(path, sizeof(path), "%s/%s", directory, name);
    file_ = open(path, O_RDWR | O_CREAT | O_EXCL, 0600);
    if (file_ < 0) { fail(errno); return ESP_FAIL; }
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_ = {};
        std::strcpy(status_.filename, name);
    }
    buffered_ = 0;
    ULog log(append_message, this);
    if (!log.begin(esp_timer_get_time()) || !flush(true)) { close(file_); file_ = -1; return ESP_FAIL; }
    static_assert(std::is_trivially_copyable_v<Event>);
    queue_ = xQueueCreate(CONFIG_CAR_LOG_QUEUE_RECORDS, sizeof(Event));
    stopped_ = xSemaphoreCreateBinary();
    if (!queue_ || !stopped_) {
        if (queue_) vQueueDelete(queue_);
        if (stopped_) vSemaphoreDelete(stopped_);
        queue_ = nullptr; stopped_ = nullptr;
        close(file_); file_ = -1; fail(ENOMEM); return ESP_ERR_NO_MEM;
    }
    stopping_.store(false);
    if (xTaskCreatePinnedToCore(task_entry, "sd_logger", 8192, this, 2, &task_, 1) != pdPASS) {
        vQueueDelete(queue_); vSemaphoreDelete(stopped_); queue_ = nullptr; stopped_ = nullptr;
        close(file_); file_ = -1; fail(ENOMEM); return ESP_ERR_NO_MEM;
    }
    { std::lock_guard<std::mutex> lock(status_mutex_); status_.recording = true; }
    accepting_.store(true);
    return ESP_OK;
}

void Logger::stop() {
    accepting_.store(false);
    if (!task_) return;
    stopping_.store(true);
    xSemaphoreTake(stopped_, portMAX_DELAY);
    task_ = nullptr;
    vQueueDelete(queue_); queue_ = nullptr;
    vSemaphoreDelete(stopped_); stopped_ = nullptr;
}

Status Logger::status() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    auto result = status_;
    result.dropped_records = dropped_.load();
    return result;
}

void Logger::fail(int error) {
    accepting_.store(false);
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.recording = false;
    status_.error = error ? error : EIO;
}

void Logger::enqueue(const Event& event) {
    if (!accepting_.load()) return;
    if (xQueueSend(queue_, &event, 0) != pdTRUE) {
        ++dropped_;
        int64_t unset = 0;
        const auto timestamp = std::visit([](const auto& sample) { return sample.timestamp_us; }, event);
        dropout_started_us_.compare_exchange_strong(unset, timestamp);
    }
}

void Logger::observe_control(void* context, const vehicle::ControlSample& sample) {
    auto& logger = *static_cast<Logger*>(context);
    if (!logger.accepting_.load()) return;
    logger.enqueue(Event{sample});
}

void Logger::record_power(uint64_t timestamp_us, const PowerMonitor::Reading& reading) {
    if (!accepting_.load()) return;
    enqueue(Event{PowerSample{timestamp_us, reading}});
}

void Logger::observe_power(void* context, uint64_t timestamp_us, const PowerMonitor::Reading& reading) {
    static_cast<Logger*>(context)->record_power(timestamp_us, reading);
}

void Logger::observe_accel(void* context, const imu::AccelData& sample) {
    static_cast<Logger*>(context)->enqueue(Event{sample});
}

void Logger::observe_gyro(void* context, const imu::GyroData& sample) {
    static_cast<Logger*>(context)->enqueue(Event{sample});
}

void Logger::observe_quat(void* context, const imu::QuatData& sample) {
    static_cast<Logger*>(context)->enqueue(Event{sample});
}

void Logger::observe_euler(void* context, const imu::EulerData& sample) {
    static_cast<Logger*>(context)->enqueue(Event{sample});
}

void Logger::observe_gps(void* context, const nav::GpsSample& sample) {
    static_cast<Logger*>(context)->enqueue(Event{sample});
}

void Logger::observe_rpm(void* context, size_t motor, const dshot::ErpmSample& sample) {
    if (motor < vehicle::motor_count && sample.updated_at_us > 0)
        static_cast<Logger*>(context)->enqueue(Event{RpmSample{uint64_t(sample.updated_at_us), motor, sample}});
}

bool Logger::append_message(void* context, const void* data, size_t size) {
    return static_cast<Logger*>(context)->append(data, size);
}

bool Logger::append(const void* data, size_t size) {
    if (size > buffer_.size()) { fail(EINVAL); return false; }
    if (buffered_ + size > buffer_.size() && !flush(false)) return false;
    std::memcpy(buffer_.data() + buffered_, data, size);
    buffered_ += size;
    return true;
}

bool Logger::flush(bool sync) {
    size_t offset = 0;
    while (offset < buffered_) {
        const auto count = write(file_, buffer_.data() + offset, buffered_ - offset);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) { fail(count < 0 ? errno : EIO); return false; }
        offset += size_t(count);
    }
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.bytes_written += buffered_;
    }
    buffered_ = 0;
    if (sync) {
        if (fsync(file_) != 0) { fail(errno); return false; }
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.synced_bytes = status_.bytes_written;
    }
    return true;
}

void Logger::task_entry(void* context) { static_cast<Logger*>(context)->run(); }

void Logger::run() {
    ULog log(append_message, this);
    uint32_t revision = 0;
    bool tuning_written = false, good = true;
    auto last_flush = esp_timer_get_time(), last_sync = last_flush;
    while (good && (!stopping_.load() || uxQueueMessagesWaiting(queue_))) {
        Event event;
        const bool received = xQueueReceive(queue_, &event, pdMS_TO_TICKS(100)) == pdTRUE;
        const auto now = esp_timer_get_time();
        if (const auto dropout = dropout_started_us_.exchange(0))
            good = log.dropout(uint16_t(std::clamp<int64_t>((now - dropout) / 1000, 1, 65535)));
        if (good && received) {
            good = std::visit([&](const auto& sample) {
                using T = std::decay_t<decltype(sample)>;
                if constexpr (std::is_same_v<T, vehicle::ControlSample>) {
                    if (!tuning_written || revision != sample.tuning_revision) {
                        if (!log.tuning(sample.timestamp_us, sample.tuning_revision, sample.tuning)) return false;
                        revision = sample.tuning_revision; tuning_written = true;
                    }
                    return log.control(sample);
                } else if constexpr (std::is_same_v<T, PowerSample>) return log.power(sample.timestamp_us, sample.reading);
                else if constexpr (std::is_same_v<T, imu::AccelData>) return log.accel(sample);
                else if constexpr (std::is_same_v<T, imu::GyroData>) return log.gyro(sample);
                else if constexpr (std::is_same_v<T, imu::QuatData>) return log.quat(sample);
                else if constexpr (std::is_same_v<T, imu::EulerData>) return log.euler(sample);
                else if constexpr (std::is_same_v<T, nav::GpsSample>) return log.gps(sample);
                else return log.rpm(sample.motor, sample.reading);
            }, event);
            if (good) { std::lock_guard<std::mutex> lock(status_mutex_); ++status_.records_written; }
        }
        if (good && now - last_sync >= 1000000) {
            good = flush(true); last_flush = last_sync = now;
        } else if (good && now - last_flush >= 100000) {
            good = flush(false); last_flush = now;
        }
    }
    accepting_.store(false);
    if (good) flush(true);
    close(file_); file_ = -1;
    { std::lock_guard<std::mutex> lock(status_mutex_); status_.recording = false; }
    xSemaphoreGive(stopped_);
    vTaskDelete(nullptr);
}

int Logger::open_download(const char* name, uint64_t& length) const {
    if (!valid_filename(name)) { errno = EINVAL; return -1; }
    const auto snapshot = status();
    char path[64]; std::snprintf(path, sizeof(path), "%s/%s", directory, name);
    const int file = open(path, O_RDONLY);
    if (file < 0) return -1;
    struct stat info{};
    if (fstat(file, &info) != 0) { const auto error = errno; close(file); errno = error; return -1; }
    length = std::strcmp(name, snapshot.filename) == 0 ? snapshot.synced_bytes : uint64_t(info.st_size);
    length = std::min(length, uint64_t(info.st_size));
    return file;
}

int Logger::delete_session(const char* name) const {
    if (!valid_filename(name)) { errno = EINVAL; return -1; }
    if (std::strcmp(name, status().filename) == 0) { errno = EBUSY; return -1; }
    char path[64]; std::snprintf(path, sizeof(path), "%s/%s", directory, name);
    return unlink(path);
}
}
