#pragma once

#include "esp_now_interface.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <memory>  // Include for std::unique_ptr
#include "telemetry_types.h" // Include shared types


// Forward declarations (?)
namespace sensor {
    struct ImuData;
    struct GPSData;
    struct SbusData;
}



namespace Telemetry {
    
class TelemetryManager{
public:
    struct Config {
        uint8_t peer_mac[ESP_NOW_ETH_ALEN];
        uint8_t wifi_channel{0};
        uint16_t queue_size{16}; 
    };

    using TelemetryQueueItem = Telemetry::TelemetryQueueItem;

    explicit TelemetryManager(const Config& config);
    ~TelemetryManager();

    // Delete copy and move operations
    TelemetryManager(const TelemetryManager&) = delete;
    TelemetryManager& operator=(const TelemetryManager&) = delete;
    TelemetryManager(TelemetryManager&&) = delete;
    TelemetryManager& operator=(TelemetryManager&&) = delete;

    esp_err_t init();
    esp_err_t start();
    esp_err_t stop();

    // For datamanager
    QueueHandle_t getQueue() const { return telemetry_queue_; }


private:
    static constexpr const char* TAG = "TelemetryManager";

    Config config_;
    QueueHandle_t telemetry_queue_{nullptr};
    std::unique_ptr<Telemetry::EspNowInterface> esp_now_interface_;
    bool is_initialized_{false};
    bool is_running_{false};
};
}