#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

extern "C" void app_main(void) {
    ESP_LOGI("APP_MAIN", "Hello, ESP-IDF!");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
