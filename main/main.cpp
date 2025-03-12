// #include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu.h"
#include "SBUS.h"
#include "gps.h"
#include "servo.h"
#include "esc_driver.h"
#include "vdc.h"
#include "log_monitor.h"
#include "config_manager.h"

#ifndef TAG
#define TAG "main"
#endif

// FIXME: Critical issue with memory allocation in buffer overflow scenario - needs immediate attention
// BUG: Race condition detected when multiple threads access shared resource simultaneously
// XXX: Temporary patch for compatibility with legacy systems - must be addressed before v2.0

// TODO: Implement proper error handling for network failures
// TASK: Add unit tests for new features
// PENDING: Update documentation for API changes

// NOTE: This algorithm has O(n^2) complexity - consider optimization
// INFO: Configuration must be set in environment variables

// OPTIMIZE: Database query executing full table scan
// PERF: Heavy computation in main thread - consider moving to background
// SLOW: Image processing taking too long for large files


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
    log_config.tcp_port = 8888;

    LogMonitor::instance().init(log_config);
    LogMonitor::instance().start();

    ESP_LOGI("main", "Log monitor started! Connect to WiFi SSID: %s", log_config.ap_ssid);
    ESP_LOGI("main", "Use 'nc YOUR_ESP_IP 8888' to view logs");

    #if CONFIG_SBUS_ENABLE
        sensor::SBUS::Config sbus_config = {
            .uart_num = static_cast<uart_port_t>(CONFIG_SBUS_UART_NUM),
            .uart_tx_pin = GPIO_NUM_17,
            .uart_rx_pin = static_cast<gpio_num_t>(CONFIG_SBUS_UART_RX),
            .baud_rate = 100000            // SBUS runs at 100k baud
        };
        static sensor::SBUS sbus(sbus_config);
        ESP_ERROR_CHECK(sbus.init());
        ESP_ERROR_CHECK(sbus.start());
    #endif


    // #if CONFIG_GPS_ENABLE
    //     sensor::GPS::Config gps_config = {
    //         .uart_num = static_cast<uart_port_t>(CONFIG_GPS_UART_NUM),
    //         .uart_tx_pin = static_cast<gpio_num_t>(CONFIG_GPS_UART_TX),
    //         .uart_rx_pin = static_cast<gpio_num_t>(CONFIG_GPS_UART_RX),
    //         .baud_rate = 57600, // NOTE: this specific one runs at 57600 even though the manual specifies the default is 9600 (which doesn't work). Which is why I won't add to KConfig (no im not just lazy)
    //         .rx_buffer_size = 2048,
    //         .tx_buffer_size = 1024,
    //     };
    //     static sensor::GPS gps(gps_config); // WARNING: This has to be a static or its killed because out-of-scope(?) after if-statement
    //     ESP_ERROR_CHECK(gps.init());
    //     ESP_ERROR_CHECK(gps.start());
    // #endif


    #if CONFIG_IMU_ENABLE
        sensor::IMU::Config imu_config = {
            .spi_host = SPI2_HOST,
            .spi_miso_pin = CONFIG_IMU_SPI_MISO,
            .spi_mosi_pin = CONFIG_IMU_SPI_MOSI,
            .spi_sck_pin = CONFIG_IMU_SPI_CLK,
            .spi_cs_pin = CONFIG_IMU_SPI_CS,
        };
        static sensor::IMU imu(imu_config);
        ESP_ERROR_CHECK(imu.init());
        ESP_ERROR_CHECK(imu.start());
    #endif
    //
    // Configure the steering servo
    Servo::Config servo_config = {
        .gpio_num = static_cast<gpio_num_t>(CONFIG_SERVO_OUTPUT_GPIO),  
        .freq_hz = static_cast<uint32_t>(CONFIG_SERVO_FREQUENCY_HZ)
    };

    // UPDATE: New ESC driver configuration
    EscDriver::Config esc_config;
    esc_config.motor_pins[EscDriver::MotorPosition::FRONT_RIGHT] = static_cast<gpio_num_t>(38);
    esc_config.motor_pins[EscDriver::MotorPosition::FRONT_LEFT] = static_cast<gpio_num_t>(39);
    esc_config.motor_pins[EscDriver::MotorPosition::REAR_LEFT] = static_cast<gpio_num_t>(40);
    esc_config.motor_pins[EscDriver::MotorPosition::REAR_RIGHT] = static_cast<gpio_num_t>(41);


    const VehicleDynamicsController::Config vd_config = {
        .steering_servo = servo_config,
        .esc_config = esc_config,
        .task_stack_size = 4096,
        .task_priority = 5,
        .task_period = pdMS_TO_TICKS(10)
    };

    static VehicleDynamicsController vd_controller(vd_config);
    ESP_ERROR_CHECK(vd_controller.init());
    ESP_ERROR_CHECK(vd_controller.start());

    while(true) {
        // ESP_LOGI("app_main", "Main loop running...");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}