#include <cstring>
#include "esp_now_interface.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_check.h"
#include "esp_mac.h"

namespace Telemetry {


EspNowInterface::EspNowInterface(const Config& config) {
    config_ = config;

    if (config_.telemetry_queue == nullptr) {
        telemetry_queue_ = xQueueCreate(config_.queue_size, sizeof(TelemetryQueueItem));
    } else {
            telemetry_queue_ = config_.telemetry_queue;
    }
}

EspNowInterface::~EspNowInterface() {
    stop();
    if (telemetry_queue_ && telemetry_queue_ != config_.telemetry_queue) {
        vQueueDelete(telemetry_queue_);
    }
}

esp_err_t EspNowInterface::init() {
    ESP_RETURN_ON_ERROR(initializeEspNow(), TAG, "Failed to initialize ESP-NOW");
    ESP_RETURN_ON_ERROR(registerPeer(), TAG, "Failed to initialize ESP-NOW");
    return ESP_OK;
}

esp_err_t EspNowInterface::start() {
    if (is_running_) {
        return ESP_OK;
    }

    BaseType_t task_created = xTaskCreatePinnedToCore(
        espNowTask,
        "esp_now_task",
        TASK_STACK_SIZE,
        this,
        TASK_PRIORITY,
        &task_handle_,
        1 //run on core 1 (core 0 & 1 as options)
    );

    if (task_created != pdPASS){
        ESP_LOGE(TAG, "Failed ot create ESP-NOW task");
        return ESP_FAIL;
    }

    is_running_ = true;
    return ESP_OK;
}

esp_err_t EspNowInterface::stop() {
    if (!is_running_) {
        return ESP_OK;
    }

    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    is_running_ = false;
    return ESP_OK;
}

esp_err_t EspNowInterface::initializeEspNow() {
    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "ESP-NOW init failed");

    ESP_RETURN_ON_ERROR(
        esp_now_register_send_cb(espNowSendCallback),
        TAG, "Failed to register send callback"
    );

    // ESP_RETURN_ON_ERROR(
    //     esp_now_register_recv_cb(espNowRecvCallback),
    //     TAG, "Failed to register receive callback"
    // );

    return ESP_OK;
}

esp_err_t EspNowInterface::registerPeer() {
    esp_now_peer_info_t peer_info = {};
    std::memcpy(peer_info.peer_addr, config_.peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = config_.wifi_channel;
    peer_info.encrypt = false;

    ESP_RETURN_ON_ERROR(
        esp_now_add_peer(&peer_info),
        TAG, "Failed to add peer"
    );

    return ESP_OK;
}

void EspNowInterface::espNowTask(void* parameters) {
    auto* interface = static_cast<EspNowInterface*>(parameters);
    TelemetryQueueItem item;

    while (true) {
        if (xQueueReceive(interface->telemetry_queue_, &item, portMAX_DELAY) == pdTRUE) {
            // Send the data via ESP-NOW
            esp_err_t result = esp_now_send(
                interface->config_.peer_mac,
                reinterpret_cast<const uint8_t*>(&item),
                sizeof(TelemetryQueueItem)
            );

            if (result != ESP_OK) {
                ESP_LOGW(TAG, "Failed to send ESP-NOW data: %d", result);
            }
        }
    }
}

void EspNowInterface::espNowSendCallback(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "ESP-NOW send failed to" MACSTR, MAC2STR(mac_addr));
    }
}

// void EspNowInterface::espNowRecvCallback(const uint8_t* mac_addr, const uint8_t* data, int len) {
//     // Currently we don't handle incoming data, but we could add it here if needed
//     ESP_LOGD(TAG, "Received %d bytes from " MACSTR, len, MAC2STR(mac_addr));
// }



}