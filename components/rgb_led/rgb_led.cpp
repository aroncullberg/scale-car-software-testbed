#include "rgb_led.h"
#include "esp_log.h"
#include "driver/spi_common.h"

static const char* TAG = "rgb_led";

namespace led {

Rgb& Rgb::instance()
{
    static Rgb instance;
    return instance;
}

Rgb::~Rgb()
{
    if (strip_) {
        led_strip_del(strip_);
    }
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

void Rgb::ensure_initialized()
{
    if (initialized_ || init_failed_) {
        return;
    }

    ESP_LOGI(TAG, "Initializing RGB LED on GPIO48 (SPI3)...");

    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
        init_failed_ = true;
        return;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = 48,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags = {
            .invert_out = false,
        }
    };

    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT,
        .spi_bus = SPI3_HOST,
        .flags = {
            .with_dma = true,
        },
    };

    esp_err_t ret = led_strip_new_spi_device(&strip_config, &spi_config, &strip_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED strip: %s", esp_err_to_name(ret));
        init_failed_ = true;
        return;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "RGB LED initialized");
}

void Rgb::set_color(uint8_t r, uint8_t g, uint8_t b)
{
    ensure_initialized();

    if (init_failed_) {
        return;
    }

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    if (r == current_r_ && g == current_g_ && b == current_b_) {
        xSemaphoreGive(mutex_);
        return;
    }

    if (led_strip_set_pixel(strip_, 0, r, g, b) == ESP_OK) {
        if (led_strip_refresh(strip_) == ESP_OK) {
            current_r_ = r;
            current_g_ = g;
            current_b_ = b;
        }
    }

    xSemaphoreGive(mutex_);
}

void Rgb::off()
{
    set_color(0, 0, 0);
}

} // namespace led
