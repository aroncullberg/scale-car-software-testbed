#include "data_capture/task.hpp"

#include "data_capture/ground_speed.hpp"
#include "rc_telemetry/publisher.hpp"
#include "sdkconfig.h"

namespace data_capture {

Task::Task(rclink::Receiver& receiver, const vehicle::Controller& vehicle,
           const PowerMonitor& power)
    : receiver_(receiver), vehicle_(vehicle), power_(power),
      stopped_(xSemaphoreCreateBinaryStatic(&stopped_storage_)) {}

Task::~Task() {
    stop();
    vSemaphoreDelete(stopped_);
}

esp_err_t Task::start() {
    if (task_) return ESP_ERR_INVALID_STATE;
    stop_requested_.store(false);
    last_rpm_timestamp_.fill(0);
    return xTaskCreatePinnedToCore(task_entry, "data_capture", 4096, this, 3, &task_, 1) == pdPASS ?
        ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t Task::set_rpm_observer(RpmObserver observer, void* context) {
    if (task_) return ESP_ERR_INVALID_STATE;
    rpm_observer_ = observer;
    rpm_observer_context_ = context;
    return ESP_OK;
}

void Task::stop() {
    if (!task_) return;
    stop_requested_.store(true);
    xSemaphoreTake(stopped_, portMAX_DELAY);
    task_ = nullptr;
}

void Task::task_entry(void* argument) {
    static_cast<Task*>(argument)->run();
}

void Task::run() {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t telemetry_period = pdMS_TO_TICKS(100);
    const TickType_t capture_period = rpm_observer_ ?
        pdMS_TO_TICKS(1000 / CONFIG_CAR_RPM_CAPTURE_HZ) : telemetry_period;
    TickType_t last_telemetry = last_wake - telemetry_period;
    while (!stop_requested_.load()) {
        const auto motors = vehicle_.motor_telemetry();
        if (rpm_observer_) {
            for (size_t i = 0; i < motors.size(); ++i) {
                const auto& motor = motors[i];
                const auto& sample = motor.speed;
                if (motor.available &&
                    (sample.state == dshot::ErpmState::Moving || sample.state == dshot::ErpmState::Stopped) &&
                    sample.updated_at_us > 0 && sample.updated_at_us != last_rpm_timestamp_[i]) {
                    last_rpm_timestamp_[i] = sample.updated_at_us;
                    rpm_observer_(rpm_observer_context_, i, sample);
                }
            }
        }
        const auto now = xTaskGetTickCount();
        if (TickType_t(now - last_telemetry) >= telemetry_period) {
            const auto power = power_.reading();
            const auto speed = ground_speed_cms(motors, CONFIG_CAR_WHEEL_DIAMETER_MM,
                CONFIG_CAR_MOTOR_REVOLUTIONS_PER_WHEEL_X100 / 100.0f);
            rc_telemetry::publish(receiver_, motors, power, speed);
            last_telemetry = now;
        }
        xTaskDelayUntil(&last_wake, capture_period);
    }
    xSemaphoreGive(stopped_);
    vTaskDelete(nullptr);
}

} // namespace data_capture
