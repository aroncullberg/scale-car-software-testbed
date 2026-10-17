#include <cstdio>

#include "esp_log.h"
#include "data_capture/task.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"
#include "rclink/receiver.hpp"
#include "sensors.hpp"
#include "vehicle/controller.hpp"
#include "recording/logger.hpp"
#include "web/server.hpp"

extern "C" void app_main() {
    std::printf("helo from mian\n");
    static rclink::Receiver receiver;
    static recording::Logger logger;
    const vehicle::Config vehicle_config{
        // FL, RR, RL, FR. Mounting is configurable per motor.
        {{{GPIO_NUM_18, vehicle::Mounting::Mirrored}, // FL
          {GPIO_NUM_39, vehicle::Mounting::Normal}, // RR
          {GPIO_NUM_38, vehicle::Mounting::Normal}, // RL
          {GPIO_NUM_40, vehicle::Mounting::Mirrored}}}, // FR
        {}, // Keep the DShot library's configured settings.
    };
    static vehicle::Controller car(vehicle_config, receiver);
    static data_capture::Task capture(receiver, car, PowerMonitor::instance());
    static web::Server web(car, logger);

    ESP_ERROR_CHECK(car.set_control_observer(recording::Logger::observe_control, &logger));
    ESP_ERROR_CHECK(PowerMonitor::instance().set_observer(recording::Logger::observe_power, &logger));
    ESP_ERROR_CHECK(capture.set_rpm_observer(recording::Logger::observe_rpm, &logger));
    ESP_ERROR_CHECK(receiver.start());
    ESP_ERROR_CHECK(car.start());
    ESP_ERROR_CHECK(sensors::start(logger));
    ESP_ERROR_CHECK(capture.start());
    ESP_ERROR_CHECK(web.start());

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        // const auto gyro = imu::IMU::instance().samples().gyro;
        // if (gyro.valid) {
        //     ESP_LOGI("gyro", "x=%+-7.3f y=%+-7.3f z=%+-7.3f rad/s",
        //              gyro.x_rads, gyro.y_rads, gyro.z_rads);
        // }
    }
}
