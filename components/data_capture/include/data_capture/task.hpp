#pragma once

#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "power_monitor.h"
#include "rclink/receiver.hpp"
#include "vehicle/controller.hpp"

namespace data_capture {

class Task {
public:
    Task(rclink::Receiver& receiver, const vehicle::Controller& vehicle,
         const PowerMonitor& power);
    ~Task();
    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    esp_err_t start();
    void stop();
    using RpmObserver = void (*)(void*, size_t, const dshot::ErpmSample&);
    esp_err_t set_rpm_observer(RpmObserver observer, void* context);

private:
    static void task_entry(void* argument);
    void run();

    rclink::Receiver& receiver_;
    const vehicle::Controller& vehicle_;
    const PowerMonitor& power_;
    RpmObserver rpm_observer_ = nullptr;
    void* rpm_observer_context_ = nullptr;
    std::array<int64_t, vehicle::motor_count> last_rpm_timestamp_{};
    TaskHandle_t task_ = nullptr;
    std::atomic<bool> stop_requested_{false};
    StaticSemaphore_t stopped_storage_;
    SemaphoreHandle_t stopped_;
};

} // namespace data_capture
