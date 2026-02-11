#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <inttypes.h>

#include "rclink.h"
#include "telemetry.h"
#include "driver/gpio.h"
#include "servo.h"
#include "DShotRMT.h"
#include "vehicle_controller.h"
#include "power_monitor.h"
#include "gps.h"
#include "nmea_driver.h"
#include "imu.h"
#include "bno08x.h"
#include <cmath>
#include <cstring>

#ifndef TAG
#define TAG "main"
#endif


static const char* accuracy_str(imu::Accuracy acc) {
    switch (acc) {
        case imu::Accuracy::UNRELIABLE: return "---";
        case imu::Accuracy::LOW:        return "LOW";
        case imu::Accuracy::MEDIUM:     return "MED";
        case imu::Accuracy::HIGH:       return "HI ";
        default:                        return "???";
    }
}

void logger_task(void* pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("\n");

    uint32_t cycle_count = 0;

    while (true) {
        cycle_count++;

        auto ch = rclink::Receiver::instance().get_channels();
        auto euler = imu::IMU::instance().get_euler();
        auto sat = nav::Gps::instance().getSatellite();
        auto loc = nav::Gps::instance().getLocation();
        auto hdop = nav::Gps::instance().getHDOP();

        // RC channels + Heading
        printf("RC: %4d %4d %4d %4d %4d %4d %4d %4d | ",
               ch.ch1.scaled, ch.ch2.scaled, ch.ch3.scaled, ch.ch4.scaled,
               ch.ch5.scaled, ch.ch6.scaled, ch.ch7.scaled, ch.ch8.scaled);

        if (euler.valid) {
            float heading = fmodf(euler.yaw_deg, 360.0f);
            if (heading < 0) heading += 360.0f;
            printf("HDG: %5.1f [%s] | ", heading, accuracy_str(euler.accuracy));
        } else {
            printf("HDG:   --- [---] | ");
        }

        // GPS status
        printf("GPS: %2" PRIu32 " sats ", sat.satellites);
        if (loc.isValid) {
            float lat = loc.lat_e7 / 10000000.0f;
            float lon = loc.lon_e7 / 10000000.0f;
            printf("FIX  %9.5f, %9.5f  HDOP:%3u", lat, lon, hdop);
        } else {
            printf("NO FIX");
        }

        printf("        \r");
        fflush(stdout);

        telemetry::publish(telemetry::RpmTelemetry{
            .source_id = 0,
            .rpm_values = { 1111, 2222, 3333, 4444 },
            .count = 4
        });

        vTaskDelay(pdMS_TO_TICKS(10)); // 10Hz update rate
    }
}


extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Car main application starting");

    esp_log_level_set("telemetry", ESP_LOG_DEBUG);

    telemetry::init();

    rclink::uart_pins_t rc_config{};
    rc_config.tx_pin = GPIO_NUM_17;
    rc_config.rx_pin = GPIO_NUM_16;
    rc_config.uart_num = UART_NUM_1;

    esp_err_t result = rclink::Receiver::instance().setup(rc_config);

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "RC auto-detection failed: %s", esp_err_to_name(result));
        // return;
    }

    ESP_LOGI(TAG, "waiting 200ms for BNO08x power stabilization...");
    vTaskDelay(pdMS_TO_TICKS(200));

    static imu::BNO08xBackend imu_backend;

    imu::SpiConfig imu_spi_cfg = {
        .mosi_pin = GPIO_NUM_11,
        .miso_pin = GPIO_NUM_13,
        .sclk_pin = GPIO_NUM_12,
        .cs_pin = GPIO_NUM_10,
        .int_pin = GPIO_NUM_14,
        .rst_pin = GPIO_NUM_9,
        .host = SPI2_HOST,
        .clock_speed_hz = 2000000
    };
    result = imu_backend.start(imu_spi_cfg, 5000, 5000, 40000);
    ESP_LOGI(TAG, "imu backend started");
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed: %s (continuing without it)", esp_err_to_name(result));
    } else {
        ESP_LOGI(TAG, "IMU started");
    }

    PowerMonitor::Config power_config = {
        .voltage_adc_unit = ADC_UNIT_2,
        .voltage_channel = ADC_CHANNEL_4,        // GPIO 15
        .voltage_divider_ratio = 11.1f,          // measured: 23.35V battery / 2103mV ADC
        .current_adc_unit = ADC_UNIT_1,
        .current_channel = ADC_CHANNEL_0,        // GPIO 1
        .current_scale_x10mv_per_a = 386,        // 38.6 mV/A
        .attenuation = ADC_ATTEN_DB_12,
        .samples_per_read = 16,
        .sample_interval_ms = 100,
        .battery_capacity_mah = 1050,
        .cell_count = 6,
    };

    result = PowerMonitor::instance().init(power_config);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Power monitor init failed: %s (continuing without it)", esp_err_to_name(result));
    } else {
        result = PowerMonitor::instance().start();
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Power monitor started");
        }
    }

    proto::NmeaDriver::Config nmea_config = {
        .uart_num = UART_NUM_2,
        .uart_tx_pin = GPIO_NUM_2,      // TX pin (GPS RX)
        .uart_rx_pin = GPIO_NUM_42,     // RX pin (GPS TX)
        .buad_rate = 38400,             // Standard GPS baud rate
        .rx_buffer_size = 2048,
        .tx_buffer_size = 0,
        .pattern_queue_size = 16,
        .task_stack_size = 4096,
        .task_priority = 5,
        .tick_period = pdMS_TO_TICKS(20)
    };

    static proto::NmeaDriver nmea_driver(nmea_config);
    result = nmea_driver.init();
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "NMEA driver init failed: %s (continuing without it)", esp_err_to_name(result));
    } else {
        result = nmea_driver.start();
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "NMEA driver started");
        }
    }

    // BaseType_t logger_task_result = xTaskCreate(
    //     logger_task,
    //     "logger",
    //     4096,
    //     NULL,
    //     3,
    //     NULL
    // );
    //
    // if (logger_task_result != pdPASS) {
    //     ESP_LOGW(TAG, "Failed to create logger task");
    // } else {
    //     ESP_LOGI(TAG, "Logger task started");
    // }

    Servo::Config servo_cfg = {
        .gpio_num = GPIO_NUM_47,
        .min_pulse_width_us = 1000,
        .max_pulse_width_us = 2000,
        .freq_hz = 50,
        .failsafe_position = 1000  // Center position (its stupid i know but this is rc cahnle , other is us)
    };

    static Servo steering_servo(servo_cfg);

    static DShotRMT motor_fl(GPIO_NUM_41, DSHOT600_BIDIRECTIONAL);
    static DShotRMT motor_fr(GPIO_NUM_39, DSHOT600_BIDIRECTIONAL);
    static DShotRMT motor_rl(GPIO_NUM_38, DSHOT600_BIDIRECTIONAL);
    static DShotRMT motor_rr(GPIO_NUM_40, DSHOT600_BIDIRECTIONAL);

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

    result = vehicle_controller.start();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Vehicle controller start failed: %s", esp_err_to_name(result));
        return;
    }

    while (true) {
        telemetry::publish(telemetry::RpmTelemetry{
            .source_id = 0,
            .rpm_values = { 1111, 2222, 3333, 4444 },
            .count = 4
        });
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
