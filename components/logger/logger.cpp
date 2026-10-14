#include "logger/logger.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "blackboard/blackboard.h"

namespace logger {

static const char* TAG = "logger";
static Sink* s_sink = nullptr;

static void logger_task(void*) {
    while (true) {
        size_t len = 0;
        const void* item = blackboard::ring_receive(&len, pdMS_TO_TICKS(100));
        if (!item) {
            continue;
        }
        if (s_sink) {
            s_sink->write(static_cast<const uint8_t*>(item), len);
        }
        blackboard::ring_release(item);
    }
}

esp_err_t start(Sink* sink) {
    if (!sink) {
        return ESP_ERR_INVALID_ARG;
    }
    s_sink = sink;
    if (xTaskCreate(logger_task, "logger", 4096, nullptr, 2, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "failed to create logger task");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "started");
    return ESP_OK;
}

}
