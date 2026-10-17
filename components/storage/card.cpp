#include "storage/card.hpp"

#include <memory>
#include <new>
#include "driver/sdspi_host.h"
#include "ff.h"
#include "diskio_impl.h"
#include "diskio_sdmmc.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace storage {
namespace {
esp_err_t sd_transaction_then_yield(int slot, sdmmc_command_t* command) {
    const auto error = sdspi_host_do_transaction(slot, command);
    vTaskDelay(1);
    return error;
}
}

Card::~Card() { unmount(); }

esp_err_t Card::mount(spi_host_device_t bus, gpio_num_t chip_select, int clock_khz) {
    if (card_) return ESP_ERR_INVALID_STATE;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(chip_select) || clock_khz < 400 || clock_khz > 20000)
        return ESP_ERR_INVALID_ARG;

    std::unique_ptr<sdmmc_card_t> card(new (std::nothrow) sdmmc_card_t{});
    if (!card) return ESP_ERR_NO_MEM;
    BYTE drive;
    auto error = ff_diskio_get_drive(&drive);
    if (error != ESP_OK) return error;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.do_transaction = sd_transaction_then_yield;
    host.max_freq_khz = clock_khz;
    sdspi_device_config_t device = SDSPI_DEVICE_CONFIG_DEFAULT();
    device.host_id = bus;
    device.gpio_cs = chip_select;
    error = sdspi_host_init();
    if (error != ESP_OK) return error;
    sdspi_dev_handle_t handle;
    error = sdspi_host_init_device(&device, &handle);
    if (error != ESP_OK) return error;
    host.slot = handle;
    error = sdmmc_card_init(&host, card.get());
    if (error != ESP_OK) {
        sdspi_host_remove_device(handle);
        return error;
    }

    ff_diskio_register_sdmmc(drive, card.get());
    const char volume[] = {char('0' + drive), ':', '\0'};
    const esp_vfs_fat_conf_t vfs{mount_path, volume, 4};
    FATFS* filesystem = nullptr;
    error = esp_vfs_fat_register(&vfs, &filesystem);
    if (error == ESP_OK) {
        if (f_mount(filesystem, volume, 1) == FR_OK) {
            card_ = card.release();
            drive_ = drive;
            return ESP_OK;
        }
        f_mount(nullptr, volume, 0);
        esp_vfs_fat_unregister_path(mount_path);
        error = ESP_FAIL;
    }
    ff_diskio_unregister(drive);
    sdspi_host_remove_device(handle);
    return error;
}

esp_err_t Card::unmount() {
    if (!card_) return ESP_OK;
    if (drive_ >= 0) {
        const char volume[] = {char('0' + drive_), ':', '\0'};
        const auto fat_error = f_mount(nullptr, volume, 0);
        if (fat_error != FR_OK) return ESP_FAIL;
        const auto vfs_error = esp_vfs_fat_unregister_path(mount_path);
        if (vfs_error != ESP_OK) return vfs_error;
        ff_diskio_unregister(BYTE(drive_));
        drive_ = -1;
    }
    const auto device_error = sdspi_host_remove_device(card_->host.slot);
    if (device_error != ESP_OK) return device_error;
    delete card_;
    card_ = nullptr;
    return ESP_OK;
}

} // namespace storage
