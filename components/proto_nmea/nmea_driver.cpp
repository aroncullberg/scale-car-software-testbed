//
// Created by cullb on 2025-04-27.
//

#include <algorithm>

#include "esp_timer.h"
#include "esp_check.h"
#include "esp_log.h"

#include "gps.h"

#include "nmea_driver.h"
#include "ubx.h"

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

        if (cfg_.configure_module) {
            const esp_err_t err = configureModule();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Module configuration failed (%s), using current settings",
                         esp_err_to_name(err));
            }
        }

        ESP_RETURN_ON_ERROR(enablePatternDetection(), TAG, "pattern det enable");
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
            .baud_rate = cfg_.baud_rate,
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
                                cfg_.pattern_queue_size,
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
        active_baud_ = cfg_.baud_rate;
        return ESP_OK;
    }

    esp_err_t NmeaDriver::enablePatternDetection() {
        ESP_RETURN_ON_ERROR(uart_enable_pattern_det_baud_intr(
                                cfg_.uart_num,
                                PATTERN_CHAR,
                                1,
                                9, // idle between two pattern characters
                                0, // idle after the last pattern character
                                0), // idle before the first pattern character
                            TAG, "pattern det enable");
        ESP_RETURN_ON_ERROR(uart_pattern_queue_reset(cfg_.uart_num, cfg_.pattern_queue_size),
                            TAG, "pattern det queue reset");
        uart_flush_input(cfg_.uart_num);
        xQueueReset(uart_queue_);
        return ESP_OK;
    }

    esp_err_t NmeaDriver::setUartBaud(int baud) {
        ESP_RETURN_ON_ERROR(uart_set_baudrate(cfg_.uart_num, baud), TAG, "set baud");
        active_baud_ = baud;
        uart_flush_input(cfg_.uart_num);
        return ESP_OK;
    }

    bool NmeaDriver::probeModule(int baud, uint32_t timeout_ms) {
        if (setUartBaud(baud) != ESP_OK) return false;

        uint8_t buf[128];
        const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
        size_t sentence_starts = 0;
        uint8_t prev = 0;

        // A talker ID always follows the '$', matching both bytes keeps line noise from a wrong
        // baud rate out of the result.
        while (xTaskGetTickCount() < deadline) {
            const int len = uart_read_bytes(cfg_.uart_num, buf, sizeof(buf), pdMS_TO_TICKS(100));
            for (int i = 0; i < len; ++i) {
                if (prev == '$' && buf[i] == 'G' && ++sentence_starts >= 2) return true;
                prev = buf[i];
            }
        }
        return false;
    }

    esp_err_t NmeaDriver::sendValset(const void *frame, size_t len, uint32_t ack_timeout_ms) {
        uart_flush_input(cfg_.uart_num);
        if (uart_write_bytes(cfg_.uart_num, frame, len) != static_cast<int>(len)) {
            return ESP_FAIL;
        }

        ubx::AckScanner scanner(ubx::CLASS_CFG, ubx::ID_CFG_VALSET);
        uint8_t buf[128];
        const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ack_timeout_ms);

        while (xTaskGetTickCount() < deadline) {
            const int read = uart_read_bytes(cfg_.uart_num, buf, sizeof(buf), pdMS_TO_TICKS(50));
            for (int i = 0; i < read; ++i) {
                switch (scanner.feed(buf[i])) {
                    case ubx::AckScanner::Result::Ack: return ESP_OK;
                    case ubx::AckScanner::Result::Nak: return ESP_ERR_INVALID_ARG;
                    case ubx::AckScanner::Result::Pending: break;
                }
            }
        }
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t NmeaDriver::configureModule() {
        // The module keeps its flash configuration across power cycles, so it may answer at the
        // configured baud, at the target baud (warm reset with our RAM settings still live), or at
        // the u-blox default.
        const int candidates[] = {cfg_.baud_rate, cfg_.target_baud_rate, 9600};
        int found = 0;
        for (int baud : candidates) {
            if (baud <= 0) continue;
            if (probeModule(baud, 1500)) {
                found = baud;
                break;
            }
        }
        if (found == 0) {
            ESP_LOGE(TAG, "No GPS output detected on any candidate baud rate");
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGI(TAG, "GPS responding at %d baud", found);

        // Only RMC (position, speed, course, time) and GGA (altitude, satellites, HDOP) are parsed,
        // every other sentence is dead weight on the link and directly caps the achievable rate.
        const ubx::KeyValue nav_cfg[] = {
            {ubx::KEY_MSGOUT_NMEA_GGA_UART1, 1},
            {ubx::KEY_MSGOUT_NMEA_RMC_UART1, 1},
            {ubx::KEY_MSGOUT_NMEA_GLL_UART1, 0},
            {ubx::KEY_MSGOUT_NMEA_GSA_UART1, 0},
            {ubx::KEY_MSGOUT_NMEA_GSV_UART1, 0},
            {ubx::KEY_MSGOUT_NMEA_VTG_UART1, 0},
            {ubx::KEY_MSGOUT_NMEA_ZDA_UART1, 0},
            {ubx::KEY_MSGOUT_NMEA_DTM_UART1, 0},
            {ubx::KEY_MSGOUT_NMEA_GBS_UART1, 0},
            {ubx::KEY_MSGOUT_NMEA_GNS_UART1, 0},
            {ubx::KEY_MSGOUT_NMEA_GRS_UART1, 0},
            {ubx::KEY_MSGOUT_NMEA_GST_UART1, 0},
            {ubx::KEY_MSGOUT_NMEA_VLW_UART1, 0},
            {ubx::KEY_RATE_MEAS, cfg_.meas_rate_ms},
            {ubx::KEY_RATE_NAV, 1},
        };

        uint8_t frame[UBX_FRAME_MAX];
        size_t len = ubx::buildValset(ubx::LAYER_RAM, nav_cfg,
                                      sizeof(nav_cfg) / sizeof(nav_cfg[0]),
                                      frame, sizeof(frame));
        if (len == 0) return ESP_ERR_NO_MEM;

        esp_err_t err = sendValset(frame, len, 1000);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Sentence/rate configuration not acknowledged: %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "Navigation rate set to %u ms, sentence set reduced to RMC + GGA",
                 cfg_.meas_rate_ms);

        if (cfg_.target_baud_rate <= 0 || cfg_.target_baud_rate == found) return ESP_OK;

        const ubx::KeyValue port_cfg[] = {
            {ubx::KEY_UART1_BAUDRATE, static_cast<uint32_t>(cfg_.target_baud_rate)},
        };
        len = ubx::buildValset(ubx::LAYER_RAM, port_cfg, 1, frame, sizeof(frame));
        if (len == 0) return ESP_ERR_NO_MEM;

        // The ACK is sent at the old baud rate but the module may switch before it drains, so the
        // outcome is verified by probing rather than trusted from the ACK.
        sendValset(frame, len, 300);

        if (probeModule(cfg_.target_baud_rate, 1500)) {
            ESP_LOGI(TAG, "Switched link to %d baud", cfg_.target_baud_rate);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "Module did not switch to %d baud, staying at %d",
                 cfg_.target_baud_rate, found);
        return setUartBaud(found);
    }

    void NmeaDriver::taskEntry(void *arg) {
        auto *self = static_cast<NmeaDriver *>(arg);
        self->run();
    }

    void NmeaDriver::run() {
        ESP_LOGI(TAG, "Task started at %d baud", active_baud_);

        Location location;
        Date date;
        Time time;
        Speed speed;
        Course course;
        Altitude altitude;
        Satellite satellites;

        uint64_t prev_us = 0;
        uint32_t prev_epoch_cs = UINT32_MAX;

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

            int pos = uart_pattern_pop_pos(cfg_.uart_num);
            if (pos < 0 || pos > SENTENCE_MAX) {
                ESP_LOGW(TAG, "No pattern detected or out of range (pat_pos: %d) (event.type: %d", pos, event.type);
                uart_flush_input(cfg_.uart_num);
                uart_pattern_queue_reset(cfg_.uart_num, cfg_.pattern_queue_size);
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

            ingest(buf, static_cast<size_t>(len));

            if (gps_.location.isUpdated()) {
                location.age = gps_.location.age();
                location.lat_e7 = static_cast<int32_t>(gps_.location.lat() * 1e7);
                // ESP_LOGI(TAG, "\tLocation lat: %ld", location.lat_e7);
                location.lon_e7 = static_cast<int32_t>(gps_.location.lng() * 1e7);
                // ESP_LOGI(TAG, "\tLocation lon: %ld", location.lon_e7);
                location.isValid = gps_.location.isValid();
                Gps::instance().setLocation(location);
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

                // Every sentence in an epoch repeats the same timestamp, so this counts fixes
                // rather than sentences.
                const uint32_t epoch_cs = (time.hour * 360000u) + (time.minute * 6000u) +
                                          (time.second * 100u) + time.centisecond;
                if (epoch_cs != prev_epoch_cs) {
                    const uint64_t now_us = esp_timer_get_time();
                    if (prev_us) {
                        ESP_LOGD(TAG, "Fix interval %llu ms", (now_us - prev_us) / 1000);
                    }
                    prev_us = now_us;
                    prev_epoch_cs = epoch_cs;
                }
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
