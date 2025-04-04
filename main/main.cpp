// #include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu.h"
#include "SBUS.h"
#include "gps.h"
#include "servo.h"
#include "vdc.h"
#include "log_monitor.h"
#include "config_manager.h"
#include "system_types.h"
#include <stdio.h>


#ifndef TAG
#define TAG "main"
#endif

// External PSRAM base address (example)
#define EXTERNAL_PSRAM_BASE_ADDRESS 0x3F800000

// Define the size of each heap region
#define HEAP_REGION1_SIZE 0x10000  // 64 KB
#define HEAP_REGION2_SIZE 0x20000  // 128 KB

// Define the heap regions array
static HeapRegion_t xHeapRegions[] =
{
    { (uint8_t *)EXTERNAL_PSRAM_BASE_ADDRESS, HEAP_REGION1_SIZE },
    { (uint8_t *)EXTERNAL_PSRAM_BASE_ADDRESS + HEAP_REGION1_SIZE, HEAP_REGION2_SIZE },
    { NULL, 0 } // Terminates the array
};


extern "C" [[noreturn]] void app_main(void) {
    ESP_LOGI("main", "Initializing ConfigManager");
    esp_err_t ret = ConfigManager::instance().init();
    if (ret != ESP_OK) {
        ESP_LOGE("main", "Failed to initialize ConfigManager: %d", ret);
    } else {
        ESP_LOGI("main", "ConfigManager initialized successfully");

        // Test direct access to a key
        bool imu_enabled = ConfigManager::instance().getBool("imu/enabled", true);
        ESP_LOGI("main", "Direct access to imu/enabled: %d", imu_enabled);
    }

    LogMonitor::Config log_config;
    log_config.ap_ssid = "ESP32-Monitor";
    log_config.ap_password = "password";

    LogMonitor::instance().init(log_config);
    LogMonitor::instance().start();

    ESP_LOGI("main", "Log monitor started! Connect to WiFi SSID: %s", log_config.ap_ssid);
    ESP_LOGI("main", "Use 'nc YOUR_ESP_IP 8888' to view logs");

    #if CONFIG_SBUS_ENABLE
        sensor::SBUS::Config sbus_config = {
            .uart_num = static_cast<uart_port_t>(CONFIG_SBUS_UART_NUM),
            .uart_tx_pin = GPIO_NUM_17,
            .uart_rx_pin = static_cast<gpio_num_t>(CONFIG_SBUS_UART_RX),
            .baud_rate = 100000,            // SBUS runs at 100k baud
            .targetFreq = Frequency::F111Hz // Think sbus is
        };
        static sensor::SBUS sbus(sbus_config);
        ESP_ERROR_CHECK(sbus.init());
        ESP_ERROR_CHECK(sbus.start());
    #endif


    #if CONFIG_GPS_ENABLE
        sensor::GPS::Config gps_config = {
            .uart_num = static_cast<uart_port_t>(CONFIG_GPS_UART_NUM),
            .uart_tx_pin = static_cast<gpio_num_t>(CONFIG_GPS_UART_TX),
            .uart_rx_pin = static_cast<gpio_num_t>(CONFIG_GPS_UART_RX),
            .baud_rate = 38400, // NOTE: this specific one runs at 57600 even though the manual specifies the default is 9600 (which doesn't work). Which is why I won't add to KConfig (no im not just lazy)
            .rx_buffer_size = 2048,
            .tx_buffer_size = 1024,
            .targetFreq = Frequency::F62Hz
        };
        static sensor::GPS gps(gps_config); // WARNING: This has to be a static or its killed because out-of-scope(?) after if-statement
        ESP_ERROR_CHECK(gps.init());
        ESP_ERROR_CHECK(gps.start());
    #endif

    #if CONFIG_IMU_ENABLE
        sensor::IMU::Config imu_config = {
            .spi_host = SPI2_HOST,
            .spi_miso_pin = CONFIG_IMU_SPI_MISO,
            .spi_mosi_pin = CONFIG_IMU_SPI_MOSI,
            .spi_sck_pin = CONFIG_IMU_SPI_CLK,
            .spi_cs_pin = CONFIG_IMU_SPI_CS,
            .targetFreq = Frequency::F100Hz
        };
        static sensor::IMU imu(imu_config);
        ESP_ERROR_CHECK(imu.init());
        ESP_ERROR_CHECK(imu.start());
    #endif

    Servo::Config servo_config;
    servo_config.gpio_num = static_cast<gpio_num_t>(CONFIG_SERVO_OUTPUT_GPIO);
    servo_config.freq_hz = static_cast<uint32_t>(CONFIG_SERVO_FREQUENCY_HZ);
    servo_config.min_pulse_width_us = static_cast<uint32_t>(1000);
    servo_config.max_pulse_width_us = static_cast<uint32_t>(2000);

    VehicleDynamicsController::Config vd_config;
    vd_config.motors_config.front_left_pin = static_cast<gpio_num_t>(38);
    vd_config.motors_config.front_right_pin = static_cast<gpio_num_t>(39);
    vd_config.motors_config.rear_left_pin = static_cast<gpio_num_t>(40);
    vd_config.motors_config.rear_right_pin = static_cast<gpio_num_t>(41);
    vd_config.motors_config.dshot_mode = DSHOT300_BIDIRECTIONAL;
    vd_config.servo_config = servo_config;
    vd_config.task_stack_size = 8162;
    vd_config.task_priority = 7;
    vd_config.frequency = Frequency::F62Hz;

    static VehicleDynamicsController vd_controller(vd_config);
    ESP_ERROR_CHECK(vd_controller.init());
    ESP_ERROR_CHECK(vd_controller.start());

    while(true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}