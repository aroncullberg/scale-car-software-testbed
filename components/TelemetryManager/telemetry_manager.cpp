#include <cstring>
#include "telemetry_manager.h"
#include "esp_log.h"
#include "esp_check.h"

namespace Telemetry {

TelemetryManager::TelemetryManager(const Config& config)
    : config_(config) {
}

TelemetryManager::~TelemetryManager() {
    stop();
    if (telemetry_queue_) {
        vQueueDelete(telemetry_queue_);
    }
}

esp_err_t TelemetryManager::init() {
    if (is_initialized_) {
        return ESP_OK;
    }

    // Create the telemetry queue
    telemetry_queue_ = xQueueCreate(config_.queue_size, sizeof(TelemetryQueueItem));
    if (telemetry_queue_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create telemetry queue");
        return ESP_FAIL;
    }

    // Initialize ESP-NOW interface
    EspNowInterface::Config esp_now_config{
        .telemetry_queue = telemetry_queue_,
        .queue_size = config_.queue_size
    };
    std::memcpy(esp_now_config.peer_mac, config_.peer_mac, ESP_NOW_ETH_ALEN);
    esp_now_config.wifi_channel = config_.wifi_channel;

    esp_now_interface_ = std::make_unique<EspNowInterface>(esp_now_config);
    
    ESP_RETURN_ON_ERROR(
        esp_now_interface_->init(),
        TAG,
        "Failed to initialize ESP-NOW interface"
    );

    is_initialized_ = true;
    return ESP_OK;
}

esp_err_t TelemetryManager::start() {
    if (!is_initialized_) {
        ESP_LOGE(TAG, "Must initialize before starting");
        return ESP_ERR_INVALID_STATE;
    }

    if (is_running_) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(
        esp_now_interface_->start(),
        TAG,
        "Failed to start ESP-NOW interface"
    );

    is_running_ = true;
    return ESP_OK;
}

esp_err_t TelemetryManager::stop() {
    if (!is_running_) {
        return ESP_OK;
    }

    if (esp_now_interface_) {
        ESP_RETURN_ON_ERROR(
            esp_now_interface_->stop(),
            TAG,
            "Failed to stop ESP-NOW interface"
        );
    }

    is_running_ = false;
    return ESP_OK;
}

} // namespace Telemetry