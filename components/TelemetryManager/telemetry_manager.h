#pragma once

#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"

namespace sensor {
    struct ImuData;
    struct GPSData;
    struct SbusData;
}

namespace telemetry {
class TelemetryManager {
public:

    struct Config {
        uint8_t peer_mac[6];
        size_t queue_size{16};
        TickType_t task_period{pdMS_TO_TICKS(10)};
        TickType_t fetcher_period{pdMS_TO_TICKS(1000)}; // 1000ms default for fetching
        uint8_t esp_now_channel{0};
        uint8_t task_priority{5};
        uint8_t fetcher_priority{5};
        size_t task_stack_size{4096};
        size_t fetcher_stack_size{4096};
    };

    enum class PacketType : uint8_t {
        COMMAND = 0x01,
        TEXT = 0x02,
        SENSOR = 0x03,
        HEARTBEAT = 0x04,
    };

    static TelemetryManager& instance() {
        static TelemetryManager instance;
        return instance;
    };

    esp_err_t init(const Config& config);
    esp_err_t start();
    esp_err_t stop();

    esp_err_t queueSensorData(const sensor::ImuData& data);
    esp_err_t queueSensorData(const sensor::GPSData& data);
    esp_err_t queueSensorData(const sensor::SbusData& data);
    esp_err_t queueTextMessage(const char* msg, uint8_t severity = 0);

private:
    TelemetryManager() = default;
    ~TelemetryManager();

    // Delete copy/move
    TelemetryManager(const TelemetryManager&) = delete;
    TelemetryManager& operator =(const TelemetryManager&) = delete;
    TelemetryManager(TelemetryManager&&) = delete;
    TelemetryManager& operator=(TelemetryManager&&) = delete;

    static void telemetryTask(void* params);
    static void dataFetcherTask(void* params);
    static void espNowSendCallback(const uint8_t* mac_addr, esp_now_send_status_t status);
    static void espNowReceiveCallback(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len);

    esp_err_t initEspNow();
    esp_err_t initWifi();
    esp_err_t transmitPacket(const void* data, size_t len);

    static constexpr uint8_t MAGIC_BYTE = 0xAE;
    static constexpr const char* TAG = "TelemetryMgr";

    Config config_{};
    QueueHandle_t telemetry_queue_{nullptr};
    TaskHandle_t task_handle_{nullptr};
    TaskHandle_t fetcher_task_handle_{nullptr};
    esp_now_peer_info_t peer_info_{};
    bool is_running_{false};
};
}