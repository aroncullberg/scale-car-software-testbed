#pragma once

#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "sensor_types.h"
#include "system_types.h"

namespace sensor {
    struct ImuData;
    struct GpsData;
    // struct SbusData;
}

namespace telemetry {
class TelemetryManager {
public:

    struct Config {
        enum class LoggingMode : uint8_t {
            // NOTE: Just trying out Doxygen-style comments, want this in the end but im to lazy to do it for everything right now.
            /**
             * @brief Only outputs logs locally using ESP_LOG macros. \n
             * Messages will appear on the serial console but won't be transmitted. 
             */
            LOCAL_ONLY = 0,
            WIRELESS_ONLY = 1,
            BOTH = 2,
        };

        LoggingMode logging_mode{static_cast<LoggingMode>(CONFIG_TELEMETRY_MODE)}; // NOTE: Yes i know that this is bad but it works so shutup
        uint8_t peer_mac[6]{0};
        size_t queue_size{16};
        TickType_t task_period{pdMS_TO_TICKS(10)};
        TickType_t fetcher_period{pdMS_TO_TICKS(1000)};
        uint8_t esp_now_channel{0};
        uint8_t task_priority{5};
        uint8_t fetcher_priority{5};
        size_t task_stack_size{4096};
        size_t fetcher_stack_size{4096};
    };

    enum class PacketType : uint8_t {
        COMMAND = 0x01,
        TEXT = 0x02,
        SENSOR_IMU = 0x03,
        SENSOR_GPS = 0x04,
        SENSOR_SBUS = 0x05,
        HEARTBEAT = 0x06,
        SYSTEM_STATS = 0x07,
    };
    
    

    static TelemetryManager& instance() {
        static TelemetryManager instance;
        return instance;
    };

    esp_err_t init(const Config& config);
    esp_err_t start();
    esp_err_t stop();

    esp_err_t queueSensorData(const sensor::ImuData& data);
    esp_err_t queueSensorData(const sensor::GpsData& data);
    esp_err_t queueSensorData(const sensor::SbusData& data);
    esp_err_t queueSystemStats(const SystemStats& stats);
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
    static esp_err_t initWifi();
    static esp_err_t initNVS(); // NOTE: this is requried for wifi (unsure if espnow needs it but i would guess so)
    static esp_err_t initNetworking(); // NOTE: This is requried for espnow/wifi
    esp_err_t transmitPacket(const void* data, size_t len);

    bool wirelessLogging() const {
        return config_.logging_mode == Config::LoggingMode::WIRELESS_ONLY ||
               config_.logging_mode == Config::LoggingMode::BOTH;
    }

    bool localLogging() const {
        return config_.logging_mode == Config::LoggingMode::LOCAL_ONLY ||
                config_.logging_mode == Config::LoggingMode::BOTH;
    }


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