#pragma once

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_err.h"
#include "sd_protocol_types.h"

namespace storage {

// Attaches to a bus already initialized by the application.
class Card {
public:
    static constexpr const char* mount_path = "/sdcard";
    ~Card();
    Card() = default;
    Card(const Card&) = delete;
    Card& operator=(const Card&) = delete;

    esp_err_t mount(spi_host_device_t host, gpio_num_t chip_select, int clock_khz);
    esp_err_t unmount();
    bool mounted() const { return card_ != nullptr && drive_ >= 0; }

private:
    sdmmc_card_t* card_ = nullptr;
    int drive_ = -1;
};

} // namespace storage
