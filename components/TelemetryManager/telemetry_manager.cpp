#include <cstring>
#include "telemetry_manager.h"
#include "esp_log.h"
#include "data_pool.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "inttypes.h"
#include "system_monitor.h"
// #include "imu.h"
// #include "gps.h"
// #include "sbus.h"

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
        sensor::GpsData gps;
        sensor::SbusData sbus;
        TextMessageData text;
        CompactSystemStats compact_stats;
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


esp_err_t TelemetryManager::init(const Config& config) {
    if (telemetry_queue_) {
        ESP_LOGE(TAG, "Already intialzied");
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;

    telemetry_queue_ = xQueueCreate(config_.queue_size, sizeof(TelemetryQueueItem));
    if (!telemetry_queue_) {
        ESP_LOGE(TAG, "Failed to create telemetry queue");
        return ESP_ERR_NO_MEM;
    }

    // Only initialize wireless components if needed
    if (wirelessLogging()) {
        ESP_ERROR_CHECK(initNVS());
        ESP_ERROR_CHECK(initNetworking());
        ESP_ERROR_CHECK(initEspNow());
    }

    return ESP_OK;
}

esp_err_t TelemetryManager::initNVS() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

esp_err_t TelemetryManager::initNetworking() {
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) return err;
    
    return esp_event_loop_create_default();
}




esp_err_t TelemetryManager::start() {
    if (!telemetry_queue_) {
        ESP_LOGE(TAG, "Not initialiezd");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ret;

    ret = xTaskCreatePinnedToCore(
        telemetryTask,
        "telemetry",
        config_.task_stack_size,
        this,
        config_.task_priority,
        &task_handle_,
        1 // core
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
        // NOTE: First task needs to be handled if the second one fails
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
        .type = PacketType::SENSOR_IMU,
        .timestamp = xTaskGetTickCount(),
    };
    item.data.imu = data;

    if (xQueueSendToBack(telemetry_queue_, &item, 0) != pdPASS) {
        ESP_LOGW(TAG, "Queue fill, IMU data dropped"); // TODO: lets not do drop here, lets do replace instead, new data is better 
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t TelemetryManager::queueSensorData(const sensor::GpsData& data) {
    if (!telemetry_queue_) {
        return ESP_ERR_INVALID_STATE;
    }

    TelemetryQueueItem item{
        .type = PacketType::SENSOR_GPS,
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
        .type = PacketType::SENSOR_SBUS,
        .timestamp = xTaskGetTickCount(),
    };
    item.data.sbus = data;

    if (xQueueSendToBack(telemetry_queue_, &item, 0) != pdPASS) {
        ESP_LOGW(TAG, "Queue fill, SBUS data dropped"); // TODO: lets not do drop here, lets do replace instead, new data is better 
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}


esp_err_t TelemetryManager::queueSystemStats(const SystemStats& stats) {
    if (!telemetry_queue_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Convert full stats to compact format
    CompactSystemStats compact{
        .free_heap_size = stats.free_heap_size,
        .min_free_heap = stats.min_free_heap,
        .heap_fragmentation = stats.heap_fragmentation,
        .cpu_frequency_mhz = stats.cpu_frequency_mhz,
        .cpu_temperature_c = stats.cpu_temperature_c,
        .cpu_usage_percent = stats.cpu_usage_percent,
        .task_count = stats.task_count,
        .max_task_watermark = 0,  // TODO: 
        .wifi_rssi = stats.wifi_rssi,
        .uptime_seconds = stats.uptime_seconds,
        .loop_time_us = stats.loop_time_us
    };

    // Find highest watermark across all tasks
    for (uint8_t i = 0; i < stats.task_count && i < 16; i++) {
        if (stats.tasks[i].stack_high_water_mark > compact.max_task_watermark) {
            compact.max_task_watermark = stats.tasks[i].stack_high_water_mark;
        }
    }

    TelemetryQueueItem item{
        .type = PacketType::SYSTEM_STATS,
        .timestamp = xTaskGetTickCount()
    };
    
    // Copy compact stats into the union
    memcpy(&item.data, &compact, sizeof(CompactSystemStats));

    if (xQueueSendToBack(telemetry_queue_, &item, 0) != pdPASS) {
        ESP_LOGW(TAG, "Queue full, system stats dropped");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}


esp_err_t TelemetryManager::queueTextMessage(const char* msg, uint8_t severity) {
    if (!telemetry_queue_ || !msg) {
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


// In telemetry_manager.cpp
void TelemetryManager::telemetryTask(void* params) {
    auto* manager = static_cast<TelemetryManager*>(params);
    TelemetryQueueItem item;
    static uint8_t sequence = 0;

    while (true) {
        if (xQueueReceive(manager->telemetry_queue_, &item, portMAX_DELAY) == pdPASS) {
            ESP_LOGV(TAG, "Received item type %u from queue, timestamp: %" PRIu32, 
                static_cast<unsigned>(item.type), item.timestamp);


            if (manager->localLogging()) {
                // Log based on packet type
                switch (item.type) {
                    case PacketType::SENSOR_GPS:
                        ESP_LOGI(TAG, "GPS Data - Lat: %" PRId32 ", Lon: %" PRId32 
                            ", Alt: %" PRId32 "mm, Speed: %" PRIu32 "mm/s, Sats: %u", 
                            item.data.gps.latitude,
                            item.data.gps.longitude,
                            item.data.gps.altitude_mm,
                            item.data.gps.speed_mmps,
                            item.data.gps.quality.satellites);
                        break;

                    case PacketType::SENSOR_IMU:
                        ESP_LOGI(TAG, "IMU Data - Accel(x:%.2f,y:%.2f,z:%.2f) "
                            "Gyro(x:%.2f,y:%.2f,z:%.2f)", 
                            item.data.imu.accel_x,
                            item.data.imu.accel_y,
                            item.data.imu.accel_z,
                            item.data.imu.gyro_x,
                            item.data.imu.gyro_y,
                            item.data.imu.gyro_z);
                        break;

                    case PacketType::TEXT:
                        ESP_LOGI(TAG, "Message [%u]: %s", 
                            item.data.text.severity,
                            item.data.text.text);
                        break;

                    case PacketType::HEARTBEAT:
                        ESP_LOGD(TAG, "Heartbeat packet");
                        break;

                    case PacketType::SYSTEM_STATS:
                        ESP_LOGI(TAG, "System Stats - Free Heap: %lu, CPU: %.1f MHz (%u%%), Tasks: %u", 
                            item.data.compact_stats.free_heap_size,
                            item.data.compact_stats.cpu_frequency_mhz,
                            item.data.compact_stats.cpu_usage_percent,
                            item.data.compact_stats.task_count);
                        break;
                    
                    
                    default:
                        ESP_LOGW(TAG, "Unknown packet type: %u", 
                            static_cast<unsigned>(item.type));
                        break;
                }
            }

            if (manager->wirelessLogging()) {
                
                PacketHeader header{
                    .magic = MAGIC_BYTE,
                    .type = static_cast<uint8_t>(item.type),
                    .sequence = sequence++,
                    .length = sizeof(item.data),
                    .ticks = xTaskGetTickCount()
                };

                size_t total_size = sizeof(header) + sizeof(item.data);
                uint8_t packet_buffer[sizeof(PacketHeader) + sizeof(TelemetryQueueItem::PacketData)];

                memcpy(packet_buffer, &header, sizeof(header));
                memcpy(packet_buffer + sizeof(header), &item.data, sizeof(item.data));

                esp_err_t err = manager->transmitPacket(packet_buffer, total_size);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to transmit packet type %d: %s", 
                        static_cast<int>(item.type), esp_err_to_name(err));
                }
            }
        }
    }
}

void TelemetryManager::dataFetcherTask(void* params) {
    auto* manager = static_cast<TelemetryManager*>(params);
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_stats_time = xTaskGetTickCount();

    const TickType_t STATS_UPDATE_PERIOD = pdMS_TO_TICKS(1000);

    while (true) {
        // get data from Vehicle singleton
        auto& vehicle_data = VehicleData::instance();

        uint32_t current_time = xTaskGetTickCount();

        if (current_time - vehicle_data.getImuTimestamp() < pdMS_TO_TICKS(100)) {
            manager->queueSensorData(vehicle_data.getImu());
        }

        if (current_time - vehicle_data.getGPSTimestamp() < pdMS_TO_TICKS(100)) {
            manager->queueSensorData(vehicle_data.getGPS());

            auto gps_data = vehicle_data.getGPS();
            ESP_LOGI(TAG, "GPS Satellites: %u", gps_data.quality.satellites);
            ESP_LOGI(TAG, "GPS Fix Type: %u", gps_data.quality.fix_type);
            ESP_LOGI(TAG, "GPS Lat: %ld", gps_data.latitude);
            ESP_LOGI(TAG, "GPS Lon: %ld", gps_data.longitude);
            
            if (gps_data.speed_valid) {
                ESP_LOGI(TAG, "GPS Speed: %lu", gps_data.speed_mmps);
            }
                    
            
            manager->queueSensorData(gps_data);

        }

        if (current_time - vehicle_data.getSbusTimestamp() < pdMS_TO_TICKS(100)) {
            manager->queueSensorData(vehicle_data.getSbus());
        }

        // TODO: Make it not borken
        // if ((current_time - last_stats_time) >= STATS_UPDATE_PERIOD) {
        //     // SystemStats stats;
        //     SystemStats stats = SystemMonitor::instance().getStats();
        //     manager->queueSystemStats(stats);
        //     last_stats_time = current_time;
        // }


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

    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW: %s", esp_err_to_name(err));
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