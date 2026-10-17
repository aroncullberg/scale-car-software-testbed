#include "rgb_led.h"

#include "led_strip.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/spi_periph.h"

namespace led {
namespace {
constexpr int pins[] = {5, 21, 48};
}

Controller::~Controller() { stop(); }

esp_err_t Controller::start() {
    if (strip_) return ESP_ERR_INVALID_STATE;
    for (int pin : pins) {
        auto result = gpio_set_level(static_cast<gpio_num_t>(pin), 0);
        if (result == ESP_OK) result = gpio_set_direction(static_cast<gpio_num_t>(pin), GPIO_MODE_OUTPUT);
        if (result != ESP_OK) return result;
    }
    led_strip_config_t config{};
    config.strip_gpio_num = 48;
    config.max_leds = 34;
    config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    config.led_model = LED_MODEL_WS2812;
    led_strip_spi_config_t spi{};
    spi.clk_src = SPI_CLK_SRC_DEFAULT;
    spi.spi_bus = SPI3_HOST;
    spi.flags.with_dma = true;
    auto result = led_strip_new_spi_device(&config, &spi, &strip_);
    if (result == ESP_OK) result = update(Frame{});
    if (result != ESP_OK) stop();
    return result;
}

esp_err_t Controller::refresh(int target, int count, const Color* pixels) {
    if (!strip_) return ESP_ERR_INVALID_STATE;
    // Disconnect the previous route before attaching the next strip. Clamp
    // its GPIO low first so switching the matrix does not create a high glitch.
    for (int pin : pins) {
        if (pin == target) continue;
        const auto result = gpio_set_level(static_cast<gpio_num_t>(pin), 0);
        if (result != ESP_OK) return result;
        esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    }
    esp_rom_gpio_connect_out_signal(target, spi_periph_signal[SPI3_HOST].spid_out, false, false);
    for (int i = 0; i < 34; ++i) {
        const auto result = i < count ?
            led_strip_set_pixel(strip_, i, pixels[i].red, pixels[i].green, pixels[i].blue) :
            led_strip_set_pixel(strip_, i, 0, 0, 0);
        if (result != ESP_OK) return result;
    }
    return led_strip_refresh(strip_);
}

esp_err_t Controller::update(const Frame& frame) {
    if (!strip_) return ESP_ERR_INVALID_STATE;
    if (frame_written_ && frame == last_frame_) return ESP_OK;
    frame_written_ = false;
    auto result = refresh(5, frame.front.size(), frame.front.data());
    if (result != ESP_OK) return result;
    result = refresh(21, frame.rear.size(), frame.rear.data());
    if (result != ESP_OK) return result;
    result = refresh(48, 1, &frame.status);
    if (result == ESP_OK) { last_frame_ = frame; frame_written_ = true; }
    return result;
}

void Controller::stop() {
    if (strip_) {
        update(Frame{});
        led_strip_del(strip_);
        strip_ = nullptr;
    }
    frame_written_ = false;
    for (int pin : pins) {
        gpio_set_level(static_cast<gpio_num_t>(pin), 0);
        esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    }
}

} // namespace led
