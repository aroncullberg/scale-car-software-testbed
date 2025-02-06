#include <cstring>
#include "telemetry_manager.h"
#include "esp_log.h"
#include "data_pool.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"

#include "imu.h"
#include "gps.h"
#include "sbus.h"



namespace telemetry {
    
struct TextMessageData {
    uint32_t timestamp;
    uint8_t severity;
    char text[120];
};

struct TelemetryQueueItem{
    TelemetryManager::PacketType type;
    uint32_t timestamp;
    union PacketData {
        sensor::ImuData imu;
        sensor::GPSData gps;
        sensor::SbusData sbus;
        TextMessageData text;
        PacketData() {} // union constructor fuckery
    } data;
};

// Packet header for all telemetry messages
struct __attribute__((packed)) PacketHeader {
    uint8_t magic;
    uint8_t type;
    uint8_t sequence;
    uint8_t length;
    uint32_t ticks;
};

TelemetryManager::~TelemetryManager() {
    stop();
    if (telemetry_queue_) {
        vQueueDelete(telemetry_queue_);
        telemetry_queue_ = nullptr;
    }
}


/*
MUST DO THIS BEFORE YOU init telemetrymanger

ESP_ERROR_CHECK(esp_netif_init());
ESP_ERROR_CHECK(esp_event_loop_create_default());
*/
esp_err_t TelemetryManager::init(const Config& config) {
    if (telemetry_queue_) {
        ESP_LOGE(TAG, "Already intialzied");
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;

    // Create telemetry queue
    telemetry_queue_ = xQueueCreate(config_.queue_size, sizeof(TelemetryQueueItem));
    if (!telemetry_queue_) {
        ESP_LOGE(TAG, "Failed to create telemetry queue");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = initEspNow();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initailzie ESP-NOW");
        return err;
    }

    // Put data fetcher initializer here

    return ESP_OK;
}

esp_err_t TelemetryManager::start() {
    if (!telemetry_queue_) {
        ESP_LOGE(TAG, "Not initialiezd");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(
        telemetryTask,
        "telemetry",
        config_.task_stack_size,
        this,
        config_.task_priority,
        &task_handle_,
        0 // core 1
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create telemetry task");
        return ESP_ERR_NO_MEM;
    }

    ret = xTaskCreatePinnedToCore(
        dataFetcherTask,
        "telem_fetch",
        config_.fetcher_stack_size,
        this,
        config_.fetcher_priority,
        &fetcher_task_handle_,
        0
    );

    if (ret != pdPASS) {
        // clena up first task if second fails
        if (task_handle_) {
            vTaskDelete(task_handle_);
            task_handle_ = nullptr;
        }
        ESP_LOGE(TAG, "Failed to create data fetcher task");
        return ESP_ERR_NO_MEM;
    }

    is_running_ = true;
    return ESP_OK;
}

esp_err_t TelemetryManager::stop() {
    if (!is_running_) {
        return ESP_OK;
    }

    if (task_handle_) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    if (fetcher_task_handle_) {
        vTaskDelete(fetcher_task_handle_);
        fetcher_task_handle_ = nullptr;
    }

    is_running_ = false;
    return ESP_OK;
}


esp_err_t TelemetryManager::queueSensorData(const sensor::ImuData& data) {
    if (!telemetry_queue_) {
        return ESP_ERR_INVALID_STATE;
    }

    TelemetryQueueItem item{
        .type = PacketType::SENSOR,
        .timestamp = xTaskGetTickCount(),
    };
    item.data.imu = data;

    if (xQueueSendToBack(telemetry_queue_, &item, 0) != pdPASS) {
        ESP_LOGW(TAG, "Queue fill, IMU data dropped"); // TODO: lets not do drop here, lets do replace instead, new data is better 
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t TelemetryManager::queueSensorData(const sensor::GPSData& data) {
    if (!telemetry_queue_) {
        return ESP_ERR_INVALID_STATE;
    }

    TelemetryQueueItem item{
        .type = PacketType::SENSOR,
        .timestamp = xTaskGetTickCount(),
    };
    item.data.gps = data;

    if (xQueueSendToBack(telemetry_queue_, &item, 0) != pdPASS) {
        ESP_LOGW(TAG, "Queue fill, GPS data dropped"); // TODO: lets not do drop here, lets do replace instead, new data is better 
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t TelemetryManager::queueSensorData(const sensor::SbusData& data) {
    if (!telemetry_queue_) {
        return ESP_ERR_INVALID_STATE;
    }

    TelemetryQueueItem item{
        .type = PacketType::SENSOR,
        .timestamp = xTaskGetTickCount(),
    };
    item.data.sbus = data;

    if (xQueueSendToBack(telemetry_queue_, &item, 0) != pdPASS) {
        ESP_LOGW(TAG, "Queue fill, SBUS data dropped"); // TODO: lets not do drop here, lets do replace instead, new data is better 
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t TelemetryManager::queueTextMessage(const char* msg, uint8_t severity) {
    if (telemetry_queue_ || !msg) {
        return ESP_ERR_INVALID_STATE;
    }

    TelemetryQueueItem item{
        .type = PacketType::TEXT,
        .timestamp = xTaskGetTickCount()
    };

    // Copy mesage WITH trunctation
    strncpy(item.data.text.text, msg, sizeof(item.data.text.text) - 1);
    item.data.text.text[sizeof(item.data.text.text) - 1] = '\0';
    item.data.text.severity = severity;
    item.data.text.timestamp = xTaskGetTickCount();

    if (xQueueSendToBack(telemetry_queue_, &item, 0) != pdPASS) {
        ESP_LOGW(TAG, "Queue full, text message dropped");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}


void TelemetryManager::telemetryTask(void* params) {
    auto* manager = static_cast<TelemetryManager*>(params);
    TelemetryQueueItem item;
    static uint8_t sequence = 0;


    while (true) {
        // TODO: have ticks at both addition to quque and sending
        if (xQueueReceive(manager->telemetry_queue_, &item, portMAX_DELAY) == pdPASS) {
            PacketHeader header {
                .magic = MAGIC_BYTE,
                .type = static_cast<uint8_t>(item.type),
                .sequence = sequence++,
                .length = sizeof(item.data),
                .ticks = xTaskGetTickCount()
            };

            // calculate totla packet size
            size_t total_size = sizeof(header) + sizeof(item.data);

            // temp buffer for complete packet
            uint8_t packet_buffer[sizeof(PacketHeader) + sizeof(TelemetryQueueItem::PacketData)];

            // copy header and data -> buffer
            memcpy(packet_buffer, &header, sizeof(header));
            memcpy(packet_buffer + sizeof(header), &item.data, sizeof(item.data));

            esp_err_t err = manager->transmitPacket(packet_buffer, total_size);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to transmit packet type %d: %s", static_cast<int>(item.type), esp_err_to_name(err));
            }

            
            // TODO: DO stuff with esp now
        }
    }
}

void TelemetryManager::dataFetcherTask(void* params) {
    auto* manager = static_cast<TelemetryManager*>(params);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        // get data from Vehicle singleton
        auto& vehicle_data = VehicleData::instance();

        uint32_t current_time = xTaskGetTickCount();

        if (current_time - vehicle_data.getImuTimestamp() < pdMS_TO_TICKS(100)) {
            manager->queueSensorData(vehicle_data.getImu());
        }

        if (current_time - vehicle_data.getGPSTimestamp() < pdMS_TO_TICKS(100)) {
            manager->queueSensorData(vehicle_data.getGPS());
        }

        if (current_time - vehicle_data.getSbusTimestamp() < pdMS_TO_TICKS(100)) {
            manager->queueSensorData(vehicle_data.getSbus());
        }

        vTaskDelayUntil(&last_wake_time, manager->config_.fetcher_period);
    }
}

esp_err_t TelemetryManager::transmitPacket(const void* data, size_t len) {
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = esp_now_send(peer_info_.peer_addr, static_cast<const uint8_t*>(data), len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send ESP-NOW packet: %s", esp_err_to_name(err));
    }

    return err;

}

esp_err_t TelemetryManager::initEspNow() {
    esp_err_t err = initWifi();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_now_register_send_cb(espNowSendCallback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to regsiter send callback: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_now_register_recv_cb(espNowReceiveCallback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register receive callback: %s", esp_err_to_name(err));
        return err;
    }

    memset(&peer_info_, 0, sizeof(peer_info_));
    memcpy(peer_info_.peer_addr, config_.peer_mac, 6);
    peer_info_.channel = config_.esp_now_channel;
    peer_info_.encrypt = false;

    err = esp_now_add_peer(&peer_info_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(err));
        return err;
    }


    return ESP_OK;
}

esp_err_t TelemetryManager::initWifi() {
    // Initialize wifi in station mode
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Wifi storange to ram only
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi storage: %s", esp_err_to_name(err));
        return err;
    }


    // Set wifi to station mode
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Faield to set WiFi mode:: %s", esp_err_to_name(err));
        return err;
    }

    // Wifi storange to ram only
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start wifi: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

void TelemetryManager::espNowSendCallback(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "ESP-NOW send failed to " MACSTR, MAC2STR(mac_addr));
    }
}


void TelemetryManager::espNowReceiveCallback(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len) {
    if (!esp_now_info || !data || data_len <= 0) {
        return;
    }

    // Log the sender's MAC address
    ESP_LOGD(TAG, "Received %d bytes from " MACSTR, data_len, MAC2STR(esp_now_info->src_addr));
}





}