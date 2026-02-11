//
// Created by cullb on 2025-04-27.
//

#include <algorithm>

#include "esp_timer.h"
#include "esp_check.h"
#include "esp_log.h"

#include "gps.h"
#include "telemetry.h"

#include "nmea_driver.h"

using nav::Gps;
using nav::Location;
using nav::Date;
using nav::Time;
using nav::Speed;
using nav::Course;
using nav::Altitude;
using nav::Satellite;
using nav::HDOP;

namespace proto
{
    NmeaDriver::NmeaDriver(Config &cfg) : cfg_(cfg) {
    }

    NmeaDriver::~NmeaDriver() {
        stop();
        uart_driver_delete(cfg_.uart_num);
    }

    esp_err_t NmeaDriver::init() {
        ESP_RETURN_ON_ERROR(configureUart(), TAG, "UART cfg failed");
        return ESP_OK;
    }

    esp_err_t NmeaDriver::start() {
        if (running_) return ESP_OK;

        running_ = true;
        BaseType_t ok = xTaskCreatePinnedToCore(
            taskEntry,
            "nmea_driver_task",
            cfg_.task_stack_size,
            this,
            cfg_.task_priority,
            &task_,
            1 // Core 1
        );
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "Failed to create task");
            running_ = false;
            return ESP_FAIL;
        }

        running_ = true;
        return ESP_OK;
    }

    esp_err_t NmeaDriver::stop() {
        if (!running_) return ESP_OK;

        running_ = false;
        uart_disable_pattern_det_intr(cfg_.uart_num);
        vTaskDelete(task_);
        task_ = nullptr;
        return ESP_OK;
    }


    esp_err_t NmeaDriver::configureUart() {
        const uart_config_t uart_cfg = {
            .baud_rate = cfg_.buad_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_APB,
        };
        ESP_RETURN_ON_ERROR(uart_driver_install(cfg_.uart_num,
                                cfg_.rx_buffer_size,
                                cfg_.tx_buffer_size,
                                QUEUE_LEN,
                                &uart_queue_,
                                0),
                            TAG, "driver install");
        ESP_RETURN_ON_ERROR(uart_param_config(cfg_.uart_num, &uart_cfg), TAG, "param");
        ESP_RETURN_ON_ERROR(uart_set_pin(cfg_.uart_num,
                                cfg_.uart_tx_pin,
                                cfg_.uart_rx_pin,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE),
                            TAG, "pin map");
        ESP_RETURN_ON_ERROR(uart_enable_pattern_det_baud_intr(
                                cfg_.uart_num,
                                '\n', // 10 <=> LF
                                1,
                                9, // idle BETWEEN two pattern chrs (is this in time or bytes?)
                                0, // idle AFTER the LAST pattern character
                                0), // idle BEFORE the FRIST apttern character
                            TAG, "pattern det enable");

        // this allocates memory for given length for something, see https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/api-reference/peripherals/uart.html#_CPPv424uart_pattern_queue_reset11uart_port_ti
        ESP_RETURN_ON_ERROR(uart_pattern_queue_reset(cfg_.uart_num, QUEUE_LEN), TAG, "pattern det queue reset");
        return ESP_OK;
    }

    void NmeaDriver::taskEntry(void *arg) {
        auto *self = static_cast<NmeaDriver *>(arg);
        self->run();
    }

    void NmeaDriver::run() {
        ESP_LOGI(TAG, "Task started");

        Location location;
        Date date;
        Time time;
        Speed speed;
        Course course;
        Altitude altitude;
        Satellite satellites;

        uint64_t prev_us = 0;

        while (running_) {
            uart_event_t event{};

            if (xQueueReceive(uart_queue_, &event, portMAX_DELAY) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to receive UART event");
                continue;
            }

            if (event.type != UART_PATTERN_DET) {
                if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                    ESP_LOGW(TAG, "FIFO overflow");
                    uart_flush_input(cfg_.uart_num);
                }
                continue;
            }

            // uint64_t now_us = esp_timer_get_time();
            // if (prev_us) {
            //     uint64_t delta_us = now_us - prev_us;
            //     float freq_hz = 1e6f / delta_us;
            //     ESP_LOGI(TAG, "(%04lld ms) %-4.1f Hz ", delta_us / 1000, freq_hz);
            // }
            // prev_us = now_us;

            int pos = uart_pattern_pop_pos(cfg_.uart_num);
            if (pos < 0 || pos > SENTENCE_MAX) {
                ESP_LOGW(TAG, "No pattern detected or out of range (pat_pos: %d) (event.type: %d", pos, event.type);
                uart_flush_input(cfg_.uart_num);
                uart_pattern_queue_reset(cfg_.uart_num, QUEUE_LEN);
                continue;
            }

            char buf[SENTENCE_MAX + 1];

            int32_t len = uart_read_bytes(cfg_.uart_num, buf, pos + 1, 0);
            if (len <= 0 || static_cast<size_t>(len) > sizeof(buf)) {
                ESP_LOGE(TAG, "Failed to read UART data");
                continue;
            }

            buf[len] = '\0';
            ESP_LOGD(TAG, "%*s", (int)(len - 2),  buf);

            // if (!gps_.location.isValid()) {
            //     ESP_LOGI(TAG, "%*s", (int)(len - 2),  buf);
            // }

            ingest(buf, static_cast<size_t>(len));

            {
                telemetry::FlightModeTelemetry fm{};
                snprintf(fm.mode, sizeof(fm.mode), "P:%u", (unsigned)gps_.passedChecksum());
                telemetry::publish(fm);
            }

            if (gps_.location.isUpdated()) {
                // ESP_LOGI(TAG, "Location updated");
                location.age = gps_.location.age();
                location.lat_e7 = static_cast<int32_t>(gps_.location.lat() * 1e7);
                // ESP_LOGI(TAG, "\tLocation lat: %ld", location.lat_e7);
                location.lon_e7 = static_cast<int32_t>(gps_.location.lng() * 1e7);
                // ESP_LOGI(TAG, "\tLocation lon: %ld", location.lon_e7);
                location.isValid = gps_.location.isValid();
                Gps::instance().setLocation(location);

                telemetry::GpsTelemetry telem = {
                    .latitude_1e7 = location.lat_e7,
                    .longitude_1e7 = location.lon_e7,
                    .speed_kmh = static_cast<uint16_t>(speed.speed_kmph),
                    .heading_deg = static_cast<uint16_t>(course.course_cd / 100),
                    .altitude_m = static_cast<uint16_t>(altitude.altitude_m),
                    .satellites = static_cast<uint8_t>(satellites.satellites),
                };
                telemetry::publish(telem);
            }

            if (gps_.date.isUpdated()) {
                // ESP_LOGI(TAG, "Date updated");
                date.age = gps_.date.age();
                date.year = gps_.date.year();
                // ESP_LOGI(TAG, "\tDate year: %u", date.year);
                date.month = gps_.date.month();
                // ESP_LOGI(TAG, "\tDate month: %u", date.month);
                date.day = gps_.date.day();
                // ESP_LOGI(TAG, "\tDate day: %u", date.day);
                date.isValid = gps_.date.isValid();
                Gps::instance().setDate(date);
            }

            if (gps_.time.isUpdated()) {
                // ESP_LOGI(TAG, "Time updated");
                time.age = gps_.time.age();
                time.hour = gps_.time.hour();
                time.minute = gps_.time.minute();
                time.second = gps_.time.second();
                // ESP_LOGI(TAG, "\t%2u:%2u:%2u", time.hour, time.minute, time.second);
                time.centisecond = gps_.time.centisecond();
                time.isValid = gps_.time.isValid();
                Gps::instance().setTime(time);
            }

            if (gps_.speed.isUpdated()) {
                // ESP_LOGI(TAG, "Speed updated");
                speed.knots = static_cast<int32_t>(gps_.speed.knots());
                // ESP_LOGI(TAG, "\tSpeed knots: %d", speed.knots);
                // speed.speed_mmps = static_cast<int32_t>(gps_.speed.mps() * 1000);
                // ESP_LOGI(TAG, "\tSpeed mphs: %d", speed.speed_mmps);
                speed.speed_kmph = static_cast<int32_t>(gps_.speed.kmph() * 10);
                // ESP_LOGI(TAG, "\tSpeed kmph: %d", speed.speed_kmph);

                Gps::instance().setSpeed(speed);
            }

            if (gps_.course.isUpdated()) {
                // ESP_LOGI(TAG, "Course updated");
                course.course_cd = static_cast<int32_t>(gps_.course.deg() * 100);
                // ESP_LOGI(TAG, "\tCourse: %ld", course.course_cd);
                Gps::instance().setCourse(course);
            }

            if (gps_.altitude.isUpdated()) {
                // ESP_LOGI(TAG, "Altitude updated");
                altitude.altitude_m = static_cast<int32_t>(gps_.altitude.meters());
                // ESP_LOGI(TAG, "\tAltitude: %d", altitude.altitude_m);
                Gps::instance().setAltitude(altitude);
            }

            if (gps_.satellites.isUpdated()) {
                // ESP_LOGI(TAG, "Satellites updated");
                satellites.age = gps_.satellites.age();
                // ESP_LOGI(TAG, "\tSatellites age: %d", satellites.age);
                satellites.satellites = gps_.satellites.value();
                // ESP_LOGI(TAG, "\tSatellites: %d", satellites.satellites);
                satellites.isValid = gps_.satellites.isValid();
                Gps::instance().setSatellite(satellites);
            }

            if (gps_.hdop.isUpdated()) {
                HDOP hdop = gps_.hdop.value();
                Gps::instance().setHDOP(hdop);
            }


        }
    }

    void NmeaDriver::ingest(const char *buf, size_t len) {
        for (size_t i = 0; i < len; ++i)
            gps_.encode((buf[i]));
    }
}
