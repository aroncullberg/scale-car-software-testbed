#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <inttypes.h>

#include "rclink.h"
#include "driver/gpio.h"
#include "servo.h"
#include "DShotRMT.h"
#include "vehicle_controller.h"
#include "voltage_sensor.h"
#include "gps.h"
#include "nmea_driver.h"
#include "imu.h"
#include "bno08x.h"
#include <cmath>
#include <cstring>

#ifndef TAG
#define TAG "main"
#endif


/**
 * @brief Logger task that prints RC channels and IMU data
 *
 * Prints first 8 RC channels (scaled 0-2000) and IMU data on a single line.
 * Format: CH1-8 | Accel XYZ | Gyro XYZ | Roll Pitch Yaw
 */
void logger_task(void* pvParameters)
{
    // Print header once
    vTaskDelay(pdMS_TO_TICKS(10000));
    printf("\n");
    printf("RC: CH1  CH2  CH3  CH4  CH5  CH6  CH7  CH8  | "
           "Accel:    X       Y       Z    | "
           "Gyro:    X       Y       Z    | "
           "Euler: Roll   Pitch    Yaw\n");
    printf("─────────────────────────────────────────────────"
           "─────────────────────────────────"
           "─────────────────────────────────"
           "──────────────────────────────\n");

    uint32_t cycle_count = 0;

    while (true) {
        cycle_count++;

        auto ch = rclink::Receiver::instance().get_channels();
        auto accel = imu::IMU::instance().get_accel();
        auto gyro = imu::IMU::instance().get_gyro();
        auto euler = imu::IMU::instance().get_euler();

        // RC channels (4 chars each, 0-2000)
        printf("   %4d %4d %4d %4d %4d %4d %4d %4d  | ",
               ch.ch1.scaled, ch.ch2.scaled, ch.ch3.scaled, ch.ch4.scaled,
               ch.ch5.scaled, ch.ch6.scaled, ch.ch7.scaled, ch.ch8.scaled);

        // Accel (m/s², 7 chars each with sign and 2 decimals)
        if (accel.valid) {
            printf("     %7.2f %7.2f %7.2f  | ", accel.x_ms2, accel.y_ms2, accel.z_ms2);
        } else {
            printf("        --      --      --  | ");
        }

        // Gyro (rad/s, 7 chars each with sign and 3 decimals)
        if (gyro.valid) {
            printf("     %7.3f %7.3f %7.3f  | ", gyro.x_rads, gyro.y_rads, gyro.z_rads);
        } else {
            printf("        --      --      --  | ");
        }

        // Euler (degrees, 7 chars each with sign and 1 decimal)
        if (euler.valid) {
            printf("     %7.1f %7.1f %7.1f", euler.roll_deg, euler.pitch_deg, euler.yaw_deg);
        } else {
            printf("        --      --      --");
        }

        printf("\r");
        fflush(stdout);

        // Send attitude telemetry every 10 cycles (1 second at 10Hz)
        if (cycle_count % 10 == 0 && euler.valid && rclink::Receiver::instance().supports_telemetry()) {
            rclink::AttitudeTelemetry attitude = {
                .roll_deg = euler.roll_deg,
                .pitch_deg = euler.pitch_deg,
                .yaw_deg = euler.yaw_deg
            };
            rclink::Receiver::instance().send_attitude(attitude);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 10Hz update rate
    }
}


extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Car main application starting");

    rclink::uart_pins_t rc_config{};
    rc_config.tx_pin = GPIO_NUM_17;
    rc_config.rx_pin = GPIO_NUM_16;
    rc_config.uart_num = UART_NUM_1;

    // ESP_LOGI(TAG, "Starting RC auto-detection...");
    // Use default ReceiverConfig (unlimited retries, 1000ms per protocol, 2 required frames)
    esp_err_t result = rclink::Receiver::instance().setup(rc_config);

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "RC auto-detection failed: %s", esp_err_to_name(result));
        // return;
    }
    // esp_err_t result;

    ESP_LOGI(TAG, "RC auto-detection successful! )NEW=SÖLKDFJ");

    // static imu::BNO08xBackend imu_backend;
    //
    // imu::SpiConfig imu_spi_cfg = {
    //     .mosi_pin = GPIO_NUM_11,
    //     .miso_pin = GPIO_NUM_13,
    //     .sclk_pin = GPIO_NUM_12,
    //     .cs_pin = GPIO_NUM_10,
    //     .int_pin = GPIO_NUM_14,
    //     .rst_pin = GPIO_NUM_9,
    //     .host = SPI2_HOST,
    //     .clock_speed_hz = 3000000
    // };
    // ESP_LOGI(TAG, "SPI auto-detection successful!");
    // // 400Hz gyro/accel for dynamics control, 50Hz quaternion for heading
    // result = imu_backend.start(imu_spi_cfg, 2500, 2500, 20000);
    // ESP_LOGI(TAG, "imu backend started");
    // if (result != ESP_OK) {
    //     ESP_LOGW(TAG, "IMU init failed: %s (continuing without it)", esp_err_to_name(result));
    // } else {
    //     ESP_LOGI(TAG, "IMU started");
    // }

    // VoltageSensor::Config voltage_config = {
    //     .adc_channel = ADC_CHANNEL_4,   // ADC1_CHANNEL_4
    //     .attenuation = ADC_ATTEN_DB_12, // 0-3.3V range (11dB)
    //     .resistor_r1_ohms = 82200,      // 82.2kΩ high-side resistor
    //     .resistor_r2_ohms = 10000,      // 10kΩ low-side resistor
    //     .samples_per_read = 16,
    //     .sample_interval_ms = 1000,
    //     .enable_telemetry = false,      // Telemetry handled by separate task
    //     .telemetry_interval_ms = 0,
    //     .battery_capacity_mah = 0
    // };
    //
    // result = VoltageSensor::instance().init(voltage_config);
    // if (result != ESP_OK) {
    //     ESP_LOGW(TAG, "Voltage sensor init failed: %s (continuing without it)", esp_err_to_name(result));
    // } else {
    //     result = VoltageSensor::instance().start();
    //     if (result == ESP_OK) {
    //         ESP_LOGI(TAG, "Voltage sensor started");
    //     }
    // }

    // ========================================
    // Step 1.6: Setup GPS/NMEA Driver
    // ========================================
    // proto::NmeaDriver::Config nmea_config = {
    //     .uart_num = UART_NUM_2,
    //     .uart_tx_pin = GPIO_NUM_17,      // TX pin (GPS RX)
    //     .uart_rx_pin = GPIO_NUM_18,     // RX pin (GPS TX)
    //     .buad_rate = 38400,              // Standard GPS baud rate
    //     .rx_buffer_size = 2048,
    //     .tx_buffer_size = 0,
    //     .pattern_queue_size = 16,
    //     .task_stack_size = 4096,
    //     .task_priority = 5,
    //     .tick_period = pdMS_TO_TICKS(20)
    // };
    //
    // static proto::NmeaDriver nmea_driver(nmea_config);
    // result = nmea_driver.init();
    // if (result != ESP_OK) {
    //     ESP_LOGW(TAG, "NMEA driver init failed: %s (continuing without it)", esp_err_to_name(result));
    // } else {
    //     result = nmea_driver.start();
    //     if (result == ESP_OK) {
    //         ESP_LOGI(TAG, "NMEA driver started");
    //     }
    // }

    // ========================================
    // Start Logger Task
    // ========================================
    BaseType_t logger_task_result = xTaskCreate(
        logger_task,
        "logger",
        4096,
        NULL,
        3,                  // Priority (lower than control tasks)
        NULL
    );

    if (logger_task_result != pdPASS) {
        ESP_LOGW(TAG, "Failed to create logger task");
    } else {
        ESP_LOGI(TAG, "Logger task started");
    }

    // ========================================
    // Step 2: Create Hardware Instances
    // ========================================

    Servo::Config servo_cfg = {
        .gpio_num = GPIO_NUM_21,
        .min_pulse_width_us = 1000,
        .max_pulse_width_us = 2000,
        .freq_hz = 50,
        .failsafe_position = 1000  // Center position (its stupid i know but this is rc cahnle , other is us)
    };

    static Servo steering_servo(servo_cfg);

    static DShotRMT motor_fl(GPIO_NUM_2, DSHOT600_BIDIRECTIONAL);
    static DShotRMT motor_fr(GPIO_NUM_41, DSHOT600_BIDIRECTIONAL);
    static DShotRMT motor_rl(GPIO_NUM_40, DSHOT600_BIDIRECTIONAL);
    static DShotRMT motor_rr(GPIO_NUM_42, DSHOT600_BIDIRECTIONAL);

    ESP_LOGI(TAG, "Initializing motors...");
    motor_fl.begin_UNSAFE();
    motor_fr.begin_UNSAFE();
    motor_rl.begin_UNSAFE();
    motor_rr.begin_UNSAFE();
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Motors initialized");

    vehicle::Controller::Config vehicle_cfg = {
        .steering_servo = steering_servo,
        .motor_fl = motor_fl,
        .motor_fr = motor_fr,
        .motor_rl = motor_rl,
        .motor_rr = motor_rr,
        .motor_poles = 14
    };

    static vehicle::Controller vehicle_controller(vehicle_cfg);

    result = vehicle_controller.init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Vehicle controller init failed: %s", esp_err_to_name(result));
        return;
    }

    result = vehicle_controller.start();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Vehicle controller start failed: %s", esp_err_to_name(result));
        return;
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
