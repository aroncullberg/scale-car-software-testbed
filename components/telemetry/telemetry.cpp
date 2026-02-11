#include "telemetry.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "telemetry";

namespace telemetry {

static constexpr size_t RING_DEPTH = 32;

template <typename T>
struct Channel {
    QueueHandle_t mailbox{nullptr};
    QueueHandle_t ring{nullptr};

    void create() {
        mailbox = xQueueCreate(1, sizeof(T));
        ring = xQueueCreate(RING_DEPTH, sizeof(T));
    }

    void publish(const T& data) {
        xQueueOverwrite(mailbox, &data);
        xQueueSend(ring, &data, 0);
    }

    bool poll(T& out) {
        return xQueueReceive(mailbox, &out, 0) == pdTRUE;
    }

    size_t drain_ring(T* buf, size_t max) {
        size_t count = 0;
        while (count < max && xQueueReceive(ring, &buf[count], 0) == pdTRUE) {
            count++;
        }
        return count;
    }
};

static Channel<AttitudeTelemetry> attitude_ch;
static Channel<RpmTelemetry> rpm_ch;
static Channel<GpsTelemetry> gps_ch;
static Channel<BatteryTelemetry> battery_ch;
static Channel<AirspeedTelemetry> airspeed_ch;
static Channel<FlightModeTelemetry> flight_mode_ch;
static Channel<TempTelemetry> temp_ch;
static Channel<AccelGyroTelemetry> accelgyro_ch;

static void fs_logger_task(void*) {
    while (true) {
        AttitudeTelemetry att[8];
        size_t n = attitude_ch.drain_ring(att, 8);
        if (n > 0) {
            ESP_LOGD(TAG, "drained %u attitude samples", (unsigned)n);
        }

        RpmTelemetry rpm[8];
        n = rpm_ch.drain_ring(rpm, 8);
        if (n > 0) {
            ESP_LOGD(TAG, "drained %u rpm samples", (unsigned)n);
        }

        GpsTelemetry gps[8];
        n = gps_ch.drain_ring(gps, 8);
        if (n > 0) {
            ESP_LOGD(TAG, "drained %u gps samples", (unsigned)n);
        }

        BatteryTelemetry bat[8];
        n = battery_ch.drain_ring(bat, 8);
        if (n > 0) {
            ESP_LOGD(TAG, "drained %u battery samples", (unsigned)n);
        }

        AirspeedTelemetry aspd[8];
        n = airspeed_ch.drain_ring(aspd, 8);
        if (n > 0) {
            ESP_LOGD(TAG, "drained %u airspeed samples", (unsigned)n);
        }

        FlightModeTelemetry fm[8];
        n = flight_mode_ch.drain_ring(fm, 8);
        if (n > 0) {
            ESP_LOGD(TAG, "drained %u flight_mode samples", (unsigned)n);
        }

        TempTelemetry tmp[8];
        n = temp_ch.drain_ring(tmp, 8);
        if (n > 0) {
            ESP_LOGD(TAG, "drained %u temp samples", (unsigned)n);
        }

        AccelGyroTelemetry ag[8];
        n = accelgyro_ch.drain_ring(ag, 8);
        if (n > 0) {
            ESP_LOGD(TAG, "drained %u accelgyro samples", (unsigned)n);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void init() {
    attitude_ch.create();
    rpm_ch.create();
    gps_ch.create();
    battery_ch.create();
    airspeed_ch.create();
    flight_mode_ch.create();
    temp_ch.create();
    accelgyro_ch.create();

    xTaskCreate(fs_logger_task, "telem_log", 2048, nullptr, 1, nullptr);
    ESP_LOGI(TAG, "telemetry dispatcher initialized");
}

void publish(const AttitudeTelemetry& data) { attitude_ch.publish(data); }
void publish(const RpmTelemetry& data) { rpm_ch.publish(data); }
void publish(const GpsTelemetry& data) { gps_ch.publish(data); }
void publish(const BatteryTelemetry& data) { battery_ch.publish(data); }
void publish(const AirspeedTelemetry& data) { airspeed_ch.publish(data); }
void publish(const FlightModeTelemetry& data) { flight_mode_ch.publish(data); }
void publish(const TempTelemetry& data) { temp_ch.publish(data); }
void publish(const AccelGyroTelemetry& data) { accelgyro_ch.publish(data); }

bool poll_latest(AttitudeTelemetry& out) { return attitude_ch.poll(out); }
bool poll_latest(RpmTelemetry& out) { return rpm_ch.poll(out); }
bool poll_latest(GpsTelemetry& out) { return gps_ch.poll(out); }
bool poll_latest(BatteryTelemetry& out) { return battery_ch.poll(out); }
bool poll_latest(AirspeedTelemetry& out) { return airspeed_ch.poll(out); }
bool poll_latest(FlightModeTelemetry& out) { return flight_mode_ch.poll(out); }
bool poll_latest(TempTelemetry& out) { return temp_ch.poll(out); }
bool poll_latest(AccelGyroTelemetry& out) { return accelgyro_ch.poll(out); }

}
