#include "rgb_led.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/spi_periph.h"
#include "driver/gpio.h"

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
        .max_leds = 34,      // Shared allocation max (supports GPIO 21 strip size)
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

void Rgb::refresh_strip_gpio_nolock(int target_gpio, int num_leds, const uint8_t pixels[][3])
{
    // The three pins we multiplex
    const int pins[] = {5, 21, 48};
    
    // 1. Route SPI3 MOSI output to target_gpio
    esp_rom_gpio_connect_out_signal(target_gpio, spi_periph_signal[SPI3_HOST].spid_out, false, false);

    // 2. Disconnect SPI3 MOSI from other pins and clamp them LOW
    //    IMPORTANT: Set the GPIO output register LOW *before* switching the pin's
    //    signal source from SPI MOSI to the GPIO output register. This prevents a
    //    brief HIGH glitch if the SPI MOSI line was left HIGH after the last transfer.
    for (int pin : pins) {
        if (pin != target_gpio) {
            gpio_set_level((gpio_num_t)pin, 0);
            gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
            esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
        }
    }

    // 3. Load the pixel data
    for (int i = 0; i < num_leds; i++) {
        led_strip_set_pixel(strip_, i, pixels[i][0], pixels[i][1], pixels[i][2]);
    }
    for (int i = num_leds; i < 34; i++) {
        led_strip_set_pixel(strip_, i, 0, 0, 0);
    }

    // 4. Refresh
    led_strip_refresh(strip_);
}

void Rgb::refresh_strip_gpio(int target_gpio, int num_leds, const uint8_t pixels[][3])
{
    ensure_initialized();
    if (init_failed_) {
        return;
    }

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    refresh_strip_gpio_nolock(target_gpio, num_leds, pixels);

    xSemaphoreGive(mutex_);
}

void Rgb::set_color(uint8_t r, uint8_t g, uint8_t b)
{
    ensure_initialized();
    if (init_failed_) {
        return;
    }

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    if (r == current_r_ && g == current_g_ && b == current_b_) {
        xSemaphoreGive(mutex_);
        return;
    }

    uint8_t pixels[1][3] = {{r, g, b}};
    refresh_strip_gpio_nolock(48, 1, pixels);

    current_r_ = r;
    current_g_ = g;
    current_b_ = b;

    xSemaphoreGive(mutex_);
}

void Rgb::off()
{
    set_color(0, 0, 0);
}

} // namespace led
