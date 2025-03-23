// #include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
// #include "imu.h"
// #include "SBUS.h"
// #include "gps.h"
// #include "servo.h"
// #include "esc_driver.h"
// #include "vdc.h"
// #include "log_monitor.h"
// #include "config_manager.h"
#include <DShotRMT.h>
#include <stdio.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#ifndef TAG
#define TAG "main"
#endif

struct log_entry_t {
    uint16_t throttle;
    uint32_t erpm;
    uint32_t tick;
};

DShotRMT motor(GPIO_NUM_41, DSHOT300_BIDIRECTIONAL);
TickType_t xLastWakeTime;
log_entry_t log_buf[10000] = {};
size_t log_pos = 0;


void log(uint16_t throttle, uint32_t erpm)
{
    log_buf[log_pos] = {
        .throttle = throttle,
        .erpm = erpm,
        .tick = xTaskGetTickCount(),
    };
    log_pos++;
}

void clearLog()
{
    // memset(log_buf, 0, sizeof(log_buf));
    log_pos = 0;
}

void printLog()
{
    // Get the ratio to convert eRPM to real RPM.
    // !! Use actual pole count (number of magnes on the bell) of your motor !!
    auto rpmRatio = DShotRMT::getErpmToRpmRatio(14);

    for (size_t i = 0; i < log_pos; i++)
    {
        log_entry_t cur = log_buf[i];
        float throttle = ((float)cur.throttle - DSHOT_THROTTLE_MIN) / (float)DSHOT_THROTTLE_RANGE;
        float rpm = INVALID_TELEMETRY_VALUE;
        if (cur.erpm != INVALID_TELEMETRY_VALUE)
            rpm = cur.erpm * rpmRatio;
        ESP_LOGI(TAG, "TICK: %lu, Throttle %%: %5.1f, RPM: %5.0f", cur.tick, throttle * 100, rpm);
    }
}

void rampThrottle(int start, int stop, int step)
{
    if (step == 0)
        return;

    for (int i = start; step > 0 ? i < stop : i > stop; i += step)
    {
        motor.sendThrottle(i);
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
    motor.sendThrottle(stop);
}

void rampThrottle(int runs)
{
    xLastWakeTime = xTaskGetTickCount();
    int taskCounter = 0;
    int rampMax = DSHOT_THROTTLE_RANGE * 0.3;

    ESP_LOGI(TAG, "Ramping throttle...");
    while (true)
    {
        // Ramp
        rampThrottle(DSHOT_THROTTLE_MIN + 50, rampMax, 1);
        rampThrottle(rampMax, DSHOT_THROTTLE_MIN + 50, -1);

        taskCounter++;
        if (taskCounter >= runs)
            break;
    }

    ESP_LOGI(TAG, "Done!");
}

void stepResponse()
{
    xLastWakeTime = xTaskGetTickCount();
    int taskCounter = 0;
    uint32_t erpm = 0;

    ESP_LOGI(TAG, "Running throttle/RPM step response...");

    uint16_t throttle_low = 150;
    int loops = 500;
    TickType_t loop_delay = pdMS_TO_TICKS(4);

    // eRPM can be consumed in three ways, demonstrated below
    uint16_t throttle_high = 250;
    motor.sendThrottle(throttle_low);

    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Method 1: Send throttle, then wait for telemetry");
    while (true)
    {
        // 1. First method to get eRPM - when the shortest possible delay in RPM reading is required:
        // - Send throttle, which occurs non-blocking on hardware, then
        // - Immediately wait for the telemetry reponse, which is a blocking "busy-wait" operation.

        motor.sendThrottle(throttle_low);
        motor.waitForErpm(erpm);
        log(throttle_low, erpm);

        xTaskDelayUntil(&xLastWakeTime, loop_delay);

        taskCounter++;
        if (taskCounter >= loops)
            break;
    }

    motor.sendThrottle(0);

    ESP_LOGI(TAG, "Done!");
    printLog();
    clearLog();
}

extern "C" [[noreturn]] void app_main(void) {
    ESP_LOGI(TAG, "Initializing DShot RMT...");
    motor.begin();

    xLastWakeTime = xTaskGetTickCount();

    // rampThrottle(2);

    stepResponse();

    // ESP_LOGW(TAG, "Exiting");



    // ESP_LOGI("main", "Initializing ConfigManager");
    // esp_err_t ret = ConfigManager::instance().init();
    // if (ret != ESP_OK) {
    //     ESP_LOGE("main", "Failed to initialize ConfigManager: %d", ret);
    // } else {
    //     ESP_LOGI("main", "ConfigManager initialized successfully");
    //
    //     // Test direct access to a key
    //     bool imu_enabled = ConfigManager::instance().getBool("imu/enabled", true);
    //     ESP_LOGI("main", "Direct access to imu/enabled: %d", imu_enabled);
    // }
    //
    // LogMonitor::Config log_config;
    // log_config.ap_ssid = "ESP32-Monitor";
    // log_config.ap_password = "password";
    //
    // LogMonitor::instance().init(log_config);
    // LogMonitor::instance().start();
    //
    // ESP_LOGI("main", "Log monitor started! Connect to WiFi SSID: %s", log_config.ap_ssid);
    // ESP_LOGI("main", "Use 'nc YOUR_ESP_IP 8888' to view logs");
    //
    // #if CONFIG_SBUS_ENABLE
    //     sensor::SBUS::Config sbus_config = {
    //         .uart_num = static_cast<uart_port_t>(CONFIG_SBUS_UART_NUM),
    //         .uart_tx_pin = GPIO_NUM_17,
    //         .uart_rx_pin = static_cast<gpio_num_t>(CONFIG_SBUS_UART_RX),
    //         .baud_rate = 100000            // SBUS runs at 100k baud
    //     };
    //     static sensor::SBUS sbus(sbus_config);
    //     ESP_ERROR_CHECK(sbus.init());
    //     ESP_ERROR_CHECK(sbus.start());
    // #endif
    //
    //
    // #if CONFIG_GPS_ENABLE
    //     sensor::GPS::Config gps_config = {
    //         .uart_num = static_cast<uart_port_t>(CONFIG_GPS_UART_NUM),
    //         .uart_tx_pin = static_cast<gpio_num_t>(CONFIG_GPS_UART_TX),
    //         .uart_rx_pin = static_cast<gpio_num_t>(CONFIG_GPS_UART_RX),
    //         .baud_rate = 38400, // NOTE: this specific one runs at 57600 even though the manual specifies the default is 9600 (which doesn't work). Which is why I won't add to KConfig (no im not just lazy)
    //         .rx_buffer_size = 2048,
    //         .tx_buffer_size = 1024,
    //     };
    //     static sensor::GPS gps(gps_config); // WARNING: This has to be a static or its killed because out-of-scope(?) after if-statement
    //     ESP_ERROR_CHECK(gps.init());
    //     ESP_ERROR_CHECK(gps.start());
    // #endif
    //
    //
    // #if CONFIG_IMU_ENABLE
    //     sensor::IMU::Config imu_config = {
    //         .spi_host = SPI2_HOST,
    //         .spi_miso_pin = CONFIG_IMU_SPI_MISO,
    //         .spi_mosi_pin = CONFIG_IMU_SPI_MOSI,
    //         .spi_sck_pin = CONFIG_IMU_SPI_CLK,
    //         .spi_cs_pin = CONFIG_IMU_SPI_CS,
    //     };
    //     static sensor::IMU imu(imu_config);
    //     ESP_ERROR_CHECK(imu.init());
    //     ESP_ERROR_CHECK(imu.start());
    // #endif
    // //
    // // Configure the steering servo
    // Servo::Config servo_config = {
    //     .gpio_num = static_cast<gpio_num_t>(CONFIG_SERVO_OUTPUT_GPIO),
    //     .freq_hz = static_cast<uint32_t>(CONFIG_SERVO_FREQUENCY_HZ)
    // };
    //
    // // UPDATE: New ESC driver configuration
    // EscDriver::Config esc_config;
    // esc_config.motor_pins[EscDriver::MotorPosition::FRONT_RIGHT] = static_cast<gpio_num_t>(38);
    // esc_config.motor_pins[EscDriver::MotorPosition::FRONT_LEFT] = static_cast<gpio_num_t>(39);
    // esc_config.motor_pins[EscDriver::MotorPosition::REAR_LEFT] = static_cast<gpio_num_t>(40);
    // esc_config.motor_pins[EscDriver::MotorPosition::REAR_RIGHT] = static_cast<gpio_num_t>(41);
    //
    //
    // const VehicleDynamicsController::Config vd_config = {
    //     .steering_servo = servo_config,
    //     .esc_config = esc_config,
    //     .task_stack_size = 8162,
    //     .task_priority = 7,
    //     .task_period = pdMS_TO_TICKS(8)
    // };
    //
    // static VehicleDynamicsController vd_controller(vd_config);
    // ESP_ERROR_CHECK(vd_controller.init());
    // ESP_ERROR_CHECK(vd_controller.start());
    //
    while(true) {
        ESP_LOGI("app_main", "Main loop running...");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}