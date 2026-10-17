#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <variant>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "recording/ulog.hpp"

namespace recording {
struct Status {
    bool recording = false;
    char filename[13]{};
    uint64_t bytes_written = 0, synced_bytes = 0;
    uint32_t records_written = 0, dropped_records = 0;
    int error = 0;
};

class Logger {
public:
    static constexpr const char* directory = "/sdcard/logs";
    Logger() = default;
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    esp_err_t start();
    void stop();
    Status status() const;
    static bool valid_filename(const char* filename);
    int open_download(const char* filename, uint64_t& length) const;
    int delete_session(const char* filename) const;
    static void observe_control(void* context, const vehicle::ControlSample& sample);
    static void observe_power(void* context, uint64_t timestamp_us, const PowerMonitor::Reading& reading);
    static void observe_accel(void* context, const imu::AccelData& sample);
    static void observe_gyro(void* context, const imu::GyroData& sample);
    static void observe_quat(void* context, const imu::QuatData& sample);
    static void observe_euler(void* context, const imu::EulerData& sample);
    static void observe_gps(void* context, const nav::GpsSample& sample);
    static void observe_rpm(void* context, size_t motor, const dshot::ErpmSample& sample);
    void record_power(uint64_t timestamp_us, const PowerMonitor::Reading& reading);
private:
    struct PowerSample {
        uint64_t timestamp_us;
        PowerMonitor::Reading reading;
    };
    struct RpmSample {
        uint64_t timestamp_us;
        size_t motor;
        dshot::ErpmSample reading;
    };
    using Event = std::variant<vehicle::ControlSample, PowerSample,
        imu::AccelData, imu::GyroData, imu::QuatData, imu::EulerData,
        nav::GpsSample, RpmSample>;
    void enqueue(const Event& event);
    static void task_entry(void* context);
    void run();
    static bool append_message(void* context, const void* data, size_t size);
    bool append(const void* data, size_t size);
    bool flush(bool sync);
    void fail(int error);

    QueueHandle_t queue_ = nullptr;
    TaskHandle_t task_ = nullptr;
    SemaphoreHandle_t stopped_ = nullptr;
    std::atomic<bool> accepting_{false}, stopping_{false};
    std::atomic<uint32_t> dropped_{0};
    std::atomic<int64_t> dropout_started_us_{0};
    mutable std::mutex status_mutex_;
    Status status_{};
    int file_ = -1;
    std::array<uint8_t, 4096> buffer_{};
    size_t buffered_ = 0;
};
}
