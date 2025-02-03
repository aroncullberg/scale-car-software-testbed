#pragma once

#include <cstdint>
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "telemetry_types.h" // Include shared types

namespace Telemetry {

class EspNowInterface {
    public:
        struct Config {
            uint8_t peer_mac[ESP_NOW_ETH_ALEN]{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            uint8_t wifi_channel{0}; // 0 = current channel
            TaskHandle_t* task_handle_{nullptr}; // optional for monitoring
            QueueHandle_t telemetry_queue{nullptr};
            uint16_t queue_size{16};
        };

        explicit EspNowInterface(const Config& config);
        ~EspNowInterface();

        // delete copy and move operations
        EspNowInterface(const EspNowInterface&) = delete;
        EspNowInterface& operator = (const EspNowInterface&) = delete;
        EspNowInterface(EspNowInterface&&) = delete;
        EspNowInterface& operator = (EspNowInterface&&) = delete;

        esp_err_t init();
        esp_err_t start();
        esp_err_t stop();

        QueueHandle_t getQueue() const {return telemetry_queue_; }

    private:
        static constexpr const char* TAG = "EspNowInterface";
        static constexpr const uint32_t TASK_STACK_SIZE = 4096;
        static constexpr const uint8_t TASK_PRIORITY = 5;
        static constexpr const uint8_t RUNNING_CORE = 1;

        static void espNowTask(void* parameters);
        static void espNowSendCallback(const uint8_t* mac_addr, esp_now_send_status_t status);
        // static void espNowRecvCallback(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len);

        esp_err_t initializeEspNow();
        esp_err_t registerPeer();

        Config config_;
        QueueHandle_t telemetry_queue_{nullptr};
        TaskHandle_t task_handle_{nullptr};
        bool is_running_{false};
};


}