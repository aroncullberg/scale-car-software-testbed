//
// Created by cullb on 2025-04-27.
//

#include <algorithm>

#include "esp_timer.h"

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
        const auto result = configureUart();
        if (result != ESP_OK) return result;
        configuration_error_ = cfg_.configure_module ? configureModule() : ESP_OK;
        return enablePatternDetection();
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

            running_ = false;
            return ESP_FAIL;
        }

        running_ = true;
        return ESP_OK;
    }

    esp_err_t NmeaDriver::set_observer(SampleObserver observer, void* context) {
        if (running_) return ESP_ERR_INVALID_STATE;
        observer_ = observer;
        observer_context_ = context;
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
        uart_config_t uart_cfg{};
        uart_cfg.baud_rate = cfg_.baud_rate;
        uart_cfg.data_bits = UART_DATA_8_BITS;
        uart_cfg.parity = UART_PARITY_DISABLE;
        uart_cfg.stop_bits = UART_STOP_BITS_1;
        uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_cfg.source_clk = UART_SCLK_APB;
        auto result = uart_driver_install(cfg_.uart_num, cfg_.rx_buffer_size,
            cfg_.tx_buffer_size, cfg_.pattern_queue_size, &uart_queue_, 0);
        if (result != ESP_OK) return result;
        result = uart_param_config(cfg_.uart_num, &uart_cfg);
        if (result != ESP_OK) return result;
        result = uart_set_pin(cfg_.uart_num, cfg_.uart_tx_pin, cfg_.uart_rx_pin,
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (result != ESP_OK) return result;
        active_baud_ = cfg_.baud_rate;
        return ESP_OK;
    }

    esp_err_t NmeaDriver::enablePatternDetection() {
        auto result = uart_enable_pattern_det_baud_intr(cfg_.uart_num,
            PATTERN_CHAR, 1, 9, 0, 0);
        if (result != ESP_OK) return result;
        result = uart_pattern_queue_reset(cfg_.uart_num, cfg_.pattern_queue_size);
        if (result != ESP_OK) return result;
        uart_flush_input(cfg_.uart_num);
        xQueueReset(uart_queue_);
        return ESP_OK;
    }

    esp_err_t NmeaDriver::setUartBaud(int baud) {
        const auto result = uart_set_baudrate(cfg_.uart_num, baud);
        if (result != ESP_OK) return result;
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

            return ESP_ERR_NOT_FOUND;
        }

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

            return err;
        }

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

            return ESP_OK;
        }

        return setUartBaud(found);
    }

    void NmeaDriver::taskEntry(void *arg) {
        auto *self = static_cast<NmeaDriver *>(arg);
        self->run();
    }

    void NmeaDriver::run() {

        Location location;
        Date date;
        Time time;
        Speed speed;
        Course course;
        Altitude altitude;
        Satellite satellites;

        while (running_) {
            uart_event_t event{};

            if (xQueueReceive(uart_queue_, &event, portMAX_DELAY) != pdTRUE) {

                continue;
            }

            if (event.type != UART_PATTERN_DET) {
                if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {

                    uart_flush_input(cfg_.uart_num);
                }
                continue;
            }

            int pos = uart_pattern_pop_pos(cfg_.uart_num);
            if (pos < 0 || pos > SENTENCE_MAX) {

                uart_flush_input(cfg_.uart_num);
                uart_pattern_queue_reset(cfg_.uart_num, cfg_.pattern_queue_size);
                continue;
            }

            char buf[SENTENCE_MAX + 1];

            int32_t len = uart_read_bytes(cfg_.uart_num, buf, pos + 1, 0);
            if (len <= 0 || static_cast<size_t>(len) >= sizeof(buf)) {

                continue;
            }

            buf[len] = '\0';

            ingest(buf, static_cast<size_t>(len));

            uint16_t updated_fields = 0;

            if (gps_.location.isUpdated()) {
                updated_fields |= nav::GPS_LOCATION;
                location.age = gps_.location.age();
                location.lat_e7 = static_cast<int32_t>(gps_.location.lat() * 1e7);
                location.lon_e7 = static_cast<int32_t>(gps_.location.lng() * 1e7);
                location.isValid = gps_.location.isValid();
                Gps::instance().setLocation(location);
            }

            if (gps_.date.isUpdated()) {
                updated_fields |= nav::GPS_DATE;
                date.age = gps_.date.age();
                date.year = gps_.date.year();
                date.month = gps_.date.month();
                date.day = gps_.date.day();
                date.isValid = gps_.date.isValid();
                Gps::instance().setDate(date);
            }

            if (gps_.time.isUpdated()) {
                updated_fields |= nav::GPS_TIME;
                time.age = gps_.time.age();
                time.hour = gps_.time.hour();
                time.minute = gps_.time.minute();
                time.second = gps_.time.second();
                time.centisecond = gps_.time.centisecond();
                time.isValid = gps_.time.isValid();
                Gps::instance().setTime(time);

            }

            if (gps_.speed.isUpdated()) {
                updated_fields |= nav::GPS_SPEED;
                speed.knots = static_cast<int32_t>(gps_.speed.knots());
                speed.speed_mmps = static_cast<int32_t>(gps_.speed.mps() * 1000);
                speed.speed_kmph = static_cast<int32_t>(gps_.speed.kmph() * 10);

                Gps::instance().setSpeed(speed);
            }

            if (gps_.course.isUpdated()) {
                updated_fields |= nav::GPS_COURSE;
                course.course_cd = static_cast<int32_t>(gps_.course.deg() * 100);
                Gps::instance().setCourse(course);
            }

            if (gps_.altitude.isUpdated()) {
                updated_fields |= nav::GPS_ALTITUDE;
                altitude.altitude_m = static_cast<int32_t>(gps_.altitude.meters());
                Gps::instance().setAltitude(altitude);
            }

            if (gps_.satellites.isUpdated()) {
                updated_fields |= nav::GPS_SATELLITES;
                satellites.age = gps_.satellites.age();
                satellites.satellites = gps_.satellites.value();
                satellites.isValid = gps_.satellites.isValid();
                Gps::instance().setSatellite(satellites);
            }

            if (gps_.hdop.isUpdated()) {
                updated_fields |= nav::GPS_HDOP;
                HDOP hdop = gps_.hdop.value();
                Gps::instance().setHDOP(hdop);
            }

            if (observer_ && updated_fields) {
                nav::GpsSample sample;
                sample.timestamp_us = esp_timer_get_time();
                sample.lat_e7 = location.lat_e7; sample.lon_e7 = location.lon_e7;
                sample.altitude_mm = static_cast<int32_t>(gps_.altitude.meters() * 1000);
                sample.speed_mmps = speed.speed_mmps; sample.course_cd = course.course_cd;
                sample.satellites = satellites.satellites;
                sample.location_age_ms = gps_.location.age();
                sample.hdop_centi = gps_.hdop.value();
                sample.year = date.year; sample.month = date.month; sample.day = date.day;
                sample.hour = time.hour; sample.minute = time.minute;
                sample.second = time.second; sample.centisecond = time.centisecond;
                sample.updated_fields = updated_fields;
                sample.valid_fields =
                    (gps_.location.isValid() ? nav::GPS_LOCATION : 0) |
                    (gps_.date.isValid() ? nav::GPS_DATE : 0) |
                    (gps_.time.isValid() ? nav::GPS_TIME : 0) |
                    (gps_.speed.isValid() ? nav::GPS_SPEED : 0) |
                    (gps_.course.isValid() ? nav::GPS_COURSE : 0) |
                    (gps_.altitude.isValid() ? nav::GPS_ALTITUDE : 0) |
                    (gps_.satellites.isValid() ? nav::GPS_SATELLITES : 0) |
                    (gps_.hdop.isValid() ? nav::GPS_HDOP : 0);
                observer_(observer_context_, sample);
            }

        }
    }

    void NmeaDriver::ingest(const char *buf, size_t len) {
        for (size_t i = 0; i < len; ++i)
            gps_.encode((buf[i]));
    }
}
