#include "sensors.hpp"

#include "bno08x.h"
#include "imu.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nmea_driver.h"
#include "power_monitor.h"
#include "sdkconfig.h"
#include "storage/card.hpp"
#include "storage_check.hpp"
#include "recording/logger.hpp"

namespace {
static imu::BNO08xBackend imu_backend;
static storage::Card sd_card;
constexpr auto spi_host = SPI2_HOST;
constexpr auto imu_chip_select = GPIO_NUM_10;
constexpr auto sd_chip_select = GPIO_NUM_8;

esp_err_t start_shared_spi() {
    // Enter SD SPI mode before any IMU traffic, with both devices deselected.
    for (const auto pin : {imu_chip_select, sd_chip_select}) {
        auto error = gpio_set_level(pin, 1);
        if (error != ESP_OK) return error;
        error = gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        if (error != ESP_OK) return error;
    }
    spi_bus_config_t bus{};
    bus.mosi_io_num = GPIO_NUM_11;
    bus.miso_io_num = GPIO_NUM_13;
    bus.sclk_io_num = GPIO_NUM_12;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.data4_io_num = -1;
    bus.data5_io_num = -1;
    bus.data6_io_num = -1;
    bus.data7_io_num = -1;
    bus.max_transfer_sz = 4096;
    const auto error = spi_bus_initialize(spi_host, &bus, SPI_DMA_CH_AUTO);
    if (error != ESP_OK) return error;
    return gpio_set_pull_mode(GPIO_NUM_13, GPIO_PULLUP_ONLY);
}
static proto::NmeaDriver::Config gps_config = {
    .uart_num = UART_NUM_2,
    .uart_tx_pin = GPIO_NUM_2,
    .uart_rx_pin = GPIO_NUM_42,
    .baud_rate = 38400,
    .target_baud_rate = 115200,
    .meas_rate_ms = 100,
    .configure_module = true,
    .rx_buffer_size = 2048,
    .tx_buffer_size = 0,
    .pattern_queue_size = 16,
    .task_stack_size = 4096,
    .task_priority = 5,
    .tick_period = pdMS_TO_TICKS(20),
};
static proto::NmeaDriver gps_driver(gps_config);

void initialize(void* context) {
    auto& logger = *static_cast<recording::Logger*>(context);
    ESP_ERROR_CHECK(imu::IMU::instance().set_observer({&logger,
        recording::Logger::observe_accel, recording::Logger::observe_gyro,
        recording::Logger::observe_quat, recording::Logger::observe_euler}));
    ESP_ERROR_CHECK(gps_driver.set_observer(recording::Logger::observe_gps, &logger));
    const PowerMonitor::Config power_config = {
        .voltage_adc_unit = ADC_UNIT_2,
        .voltage_channel = ADC_CHANNEL_4, // GPIO15.
        .voltage_divider_ratio = 11.1f,
        .current_adc_unit = ADC_UNIT_1,
        .current_channel = ADC_CHANNEL_0, // GPIO1.
        .current_scale_x10mv_per_a = 90, // Blueson A1: nominal AM32 gain, 9 mV/A; verify CR pin on bench.
        .attenuation = ADC_ATTEN_DB_12,
        .samples_per_read = 16,
        .sample_interval_ms = 100,
        .battery_capacity_mah = 1050,
        .cell_count = 6,
    };
    auto& power = PowerMonitor::instance();
    if (power.init(power_config) == ESP_OK) power.start();
    const auto bus_error = start_shared_spi();
    if (bus_error != ESP_OK) {
        ESP_LOGE("sensors", "SPI2 initialization failed: %s", esp_err_to_name(bus_error));
        vTaskDelete(nullptr);
        return;
    }
    const auto sd_error = sd_card.mount(spi_host, sd_chip_select, CONFIG_CAR_SD_CLOCK_KHZ);
    if (sd_error != ESP_OK)
        ESP_LOGE("sensors", "SD mount failed: %s", esp_err_to_name(sd_error));
    if (sd_card.mounted()) {
        const auto log_error = logger.start();
        if (log_error != ESP_OK) ESP_LOGE("sensors", "Recording failed: %s", esp_err_to_name(log_error));
    }
    if (gps_driver.init() == ESP_OK) gps_driver.start();
    const imu::SpiConfig spi = {
        .mosi_pin = GPIO_NUM_11,
        .miso_pin = GPIO_NUM_13,
        .sclk_pin = GPIO_NUM_12,
        .cs_pin = imu_chip_select,
        .int_pin = GPIO_NUM_14,
        .rst_pin = GPIO_NUM_9,
        .host = spi_host,
        .clock_speed_hz = 2000000,
    };
    const auto imu_error = imu_backend.start(spi, 5000, 5000, 40000);
    if (imu_error != ESP_OK)
        ESP_LOGE("sensors", "IMU initialization failed: %s", esp_err_to_name(imu_error));
    if (sd_card.mounted()) check_sd_read_write();
    vTaskDelete(nullptr);
}

} // namespace

esp_err_t sensors::start(recording::Logger& logger) {
    // Keep sensor interrupts on CPU1, separate from DShot's TX/RX interrupts.
    return xTaskCreatePinnedToCore(initialize, "sensor_start", 8192, &logger,
                                   4, nullptr, 1) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
