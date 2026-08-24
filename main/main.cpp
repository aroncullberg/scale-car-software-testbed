#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <stdio.h>

#include "bno08x.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_rom_gpio.h"
#include "esp_system.h"
#include "gps.h"
#include "imu.h"
#include "led_strip.h"
#include "motor.h"
#include "nmea_driver.h"
#include "power_monitor.h"
#include "rclink.h"
#include "servo.h"
#include "soc/gpio_sig_map.h"
#include "soc/spi_periph.h"
#include "vehicle_controller.h"
#include <cmath>
#include <cstring>
#include <blackboard/blackboard.h>
#include <logger/logger.h>
#include <tcp_sink/tcp_sink.h>

#ifndef TAG
#define TAG "main"
#endif

static void print_reset_reason(void) {
  esp_reset_reason_t reason = esp_reset_reason();
  const char *reason_str = "UNKNOWN";
  switch (reason) {
  case ESP_RST_POWERON:
    reason_str = "POWERON (Normal Power On)";
    break;
  case ESP_RST_EXT:
    reason_str = "EXT (External Pin Reset)";
    break;
  case ESP_RST_SW:
    reason_str = "SW (Software Reset / esp_restart)";
    break;
  case ESP_RST_PANIC:
    reason_str = "PANIC (Software Panic / Crash)";
    break;
  case ESP_RST_INT_WDT:
    reason_str = "INT_WDT (Interrupt Watchdog / Lockup)";
    break;
  case ESP_RST_TASK_WDT:
    reason_str = "TASK_WDT (Task Watchdog / Blocked Loop)";
    break;
  case ESP_RST_WDT:
    reason_str = "WDT (Other Watchdog Reset)";
    break;
  case ESP_RST_DEEPSLEEP:
    reason_str = "DEEPSLEEP (Deep Sleep Wakeup)";
    break;
  case ESP_RST_BROWNOUT:
    reason_str = "BROWNOUT (Voltage dropped too low!)";
    break;
  case ESP_RST_SDIO:
    reason_str = "SDIO (Reset over SDIO)";
    break;
  default:
    reason_str = "UNKNOWN";
    break;
  }
  ESP_LOGI(TAG, "================================================");
  ESP_LOGI(TAG, "LAST RESET REASON: %s", reason_str);
  ESP_LOGI(TAG, "================================================");
}

extern "C" void app_main(void) {
  print_reset_reason();

  ESP_LOGI(TAG, "Car main application starting");

  blackboard::init(); // the ELRS backend publishes RcChannels

  // static tcp_sink::TcpSink net_sink;
  // if (net_sink.start({}) == ESP_OK) {
  //   logger::start(&net_sink);
  // } else {
  //   ESP_LOGW(TAG, "TCP sink failed to start, logging disabled");
  // }

  rclink::uart_pins_t rc_config{};
  rc_config.tx_pin = GPIO_NUM_17;
  rc_config.rx_pin = GPIO_NUM_16;
  rc_config.uart_num = UART_NUM_1;

  rclink::ReceiverConfig receiver_config{};
  receiver_config.max_retries = 3;

  esp_err_t result =
      rclink::Receiver::instance().setup(rc_config, receiver_config);

  if (result != ESP_OK) {
    ESP_LOGW(TAG, "RC not detected, continuing without it");
  }

  // ESP_LOGI(TAG, "waiting 200ms for BNO08x power stabilization...");
  // vTaskDelay(pdMS_TO_TICKS(200));

  // static imu::BNO08xBackend imu_backend;

  // imu::SpiConfig imu_spi_cfg = {.mosi_pin = GPIO_NUM_11,
  //                               .miso_pin = GPIO_NUM_13,
  //                               .sclk_pin = GPIO_NUM_12,
  //                               .cs_pin = GPIO_NUM_10,
  //                               .int_pin = GPIO_NUM_14,
  //                               .rst_pin = GPIO_NUM_9,
  //                               .host = SPI2_HOST,
  //                               .clock_speed_hz = 2000000};
  // result = imu_backend.start(imu_spi_cfg, 5000, 5000, 40000);
  // ESP_LOGI(TAG, "imu backend started");
  // if (result != ESP_OK) {
  //   ESP_LOGW(TAG, "IMU init failed: %s (continuing without it)",
  //            esp_err_to_name(result));
  // } else {
  //   ESP_LOGI(TAG, "IMU started");
  // }

  // PowerMonitor::Config power_config = {
  //     .voltage_adc_unit = ADC_UNIT_2,
  //     .voltage_channel = ADC_CHANNEL_4, // GPIO 15
  //     .voltage_divider_ratio = 11.1f,   // measured: 23.35V battery / 2103mV
  //                                       // ADC
  //     .current_adc_unit = ADC_UNIT_1,
  //     .current_channel = ADC_CHANNEL_0, // GPIO 1
  //     .current_scale_x10mv_per_a = 386, // 38.6 mV/A
  //     .attenuation = ADC_ATTEN_DB_12,
  //     .samples_per_read = 16,
  //     .sample_interval_ms = 100,
  //     .battery_capacity_mah = 1050,
  //     .cell_count = 6,
  // };

  // result = PowerMonitor::instance().init(power_config);
  // if (result != ESP_OK) {
  //   ESP_LOGW(TAG, "Power monitor init failed: %s (continuing without it)",
  //            esp_err_to_name(result));
  // } else {
  //   result = PowerMonitor::instance().start();
  //   if (result == ESP_OK) {
  //     ESP_LOGI(TAG, "Power monitor started");
  //   }
  // }

  proto::NmeaDriver::Config nmea_config = {
      .uart_num = UART_NUM_2,
      .uart_tx_pin = GPIO_NUM_2,  // TX pin (GPS RX)
      .uart_rx_pin = GPIO_NUM_42, // RX pin (GPS TX)
      .baud_rate = 38400,         // Baud rate the module boots with
      .target_baud_rate = 115200, // Raised so 10 Hz fits on the link
      .meas_rate_ms = 100,
      .configure_module = true,
      .rx_buffer_size = 2048,
      .tx_buffer_size = 0,
      .pattern_queue_size = 16,
      .task_stack_size = 4096,
      .task_priority = 5,
      .tick_period = pdMS_TO_TICKS(20)};

  static proto::NmeaDriver nmea_driver(nmea_config);
  result = nmea_driver.init();
  if (result != ESP_OK) {
    ESP_LOGW(TAG, "NMEA driver init failed: %s (continuing without it)",
             esp_err_to_name(result));
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

  // Servo::Config servo_cfg = {
  //     .gpio_num = GPIO_NUM_47,
  //     .min_pulse_width_us = 500,
  //     .max_pulse_width_us = 2500,
  //     .freq_hz = 50,
  //     .failsafe_position = 1000 // Center position (its stupid i know but
  //                               // this is rc cahnle , other is us)
  // };

  // static Servo steering_servo(servo_cfg);

  // Servo::Config aero_cfg = {.gpio_num = GPIO_NUM_4,
  //                           .min_pulse_width_us = 1250,
  //                           .max_pulse_width_us = 2300,
  //                           .freq_hz = 50,
  //                           .failsafe_position = 1000};

  // static Servo aero_servo(aero_cfg);
  // aero_servo.setRange(100);
  // aero_servo.setInvert(false);

  // Servo::Config aux1_cfg = {.gpio_num = GPIO_NUM_6,
  //                           .min_pulse_width_us = 700,
  //                           .max_pulse_width_us = 2500,
  //                           .freq_hz = 50,
  //                           .failsafe_position = 1000,
  //                           .group_id = 1};
  // static Servo aux_servo1(aux1_cfg);
  // aux_servo1.setInvert(false);
  // aux_servo1.setRange(100);

  // Servo::Config aux2_cfg = {.gpio_num = GPIO_NUM_7,
  //                           .min_pulse_width_us = 500,
  //                           .max_pulse_width_us = 2300,
  //                           .freq_hz = 50,
  //                           .failsafe_position = 1000,
  //                           .group_id = 1};
  // static Servo aux_servo2(aux2_cfg);
  // aux_servo2.setInvert(true);
  // aux_servo2.setRange(100);

  // static motor::Motor motor_fl(
  //     {.gpio = GPIO_NUM_41, .mounting = motor::Mounting::NORMAL});
  // static motor::Motor motor_fr(
  //     {.gpio = GPIO_NUM_39, .mounting = motor::Mounting::NORMAL});
  // static motor::Motor motor_rl(
  //     {.gpio = GPIO_NUM_38, .mounting = motor::Mounting::MIRRORED});
  // static motor::Motor motor_rr(
  //     {.gpio = GPIO_NUM_40, .mounting = motor::Mounting::NORMAL});

  // vehicle::Controller::Config vehicle_cfg = {
  //     .steering_servo = steering_servo,
  //     .aero_servo = aero_servo,
  //     .aux_servo1 = aux_servo1,
  //     .aux_servo2 = aux_servo2,
  //     .motor_fl = motor_fl,
  //     .motor_fr = motor_fr,
  //     .motor_rl = motor_rl,
  //     .motor_rr = motor_rr,
  // };

  // static vehicle::Controller vehicle_controller(vehicle_cfg);

  // result = vehicle_controller.start();
  // if (result != ESP_OK) {
  //   ESP_LOGE(TAG, "Vehicle controller start failed: %s",
  //            esp_err_to_name(result));
  //   return;
  // }

  // --- Single-motor proof of concept: bypasses vehicle_controller entirely.
  // Same gating as Controller::process_rc_input (ch1 low + ch5 high to arm,
  // ch5 low to disarm), but drive-only, no brake/gear/3D-mode-limit logic.
  // Splits the two halves of "bidirectional": proper inverted idle-high DShot
  // with inverted CRC, but no half-duplex receive, so the line stays push-pull.
  // If the ESC arms here, the fault is the open-drain receive window.
  static motor::Motor test_motor({.gpio = GPIO_NUM_41,
                                  .mounting = motor::Mounting::NORMAL,
                                  .inverted = true,
                                  .telemetry = false});

  ESP_LOGI(TAG, "Initializing test motor on GPIO 41...");
  esp_err_t motor_result = test_motor.init();
  if (motor_result != ESP_OK) {
    ESP_LOGE(TAG, "Motor init failed: %s", esp_err_to_name(motor_result));
  }

  bool armed = false;

  while (true) {
    auto &receiver = rclink::Receiver::instance();

    if (!receiver.valid_data()) {
      if (armed) {
        armed = false;
        test_motor.disarm();
        ESP_LOGW(TAG, "RC lost, disarmed");
      }
      ESP_LOGI(TAG, "RC no valid data");
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    const rclink::RcChannels ch = receiver.get_channels();

    if (ch.ch1.scaled < 25 && !armed && ch.ch5.scaled > 1975) {
      armed = true;
      test_motor.arm();
      ESP_LOGI(TAG, "ARM requested");
    } else if (ch.ch5.scaled <= 1975 && armed) {
      armed = false;
      test_motor.disarm();
      ESP_LOGI(TAG, "DISARMED");
    }

    bool ret = false;
    if (armed) {
      float throttle = ch.ch1.scaled / 2000.0f;
      ret = test_motor.drive(throttle);
    }

    motor::RpmSample rpm = test_motor.rpm();
    motor::DshotDriver::RxStats rx = test_motor.rx_stats();
    ESP_LOGI(TAG,
             "ch1=%u ch5=%u armed=%d ret=%d state=%d rpm=%ld status=%d | "
             "rx_isr=%lu syms=%lu decode_fail=%lu arm_fail=%lu",
             ch.ch1.scaled, ch.ch5.scaled, armed, ret,
             static_cast<int>(test_motor.state()),
             static_cast<long>(rpm.rpm), static_cast<int>(rpm.status),
             static_cast<unsigned long>(rx.rx_isr),
             static_cast<unsigned long>(rx.rx_symbols),
             static_cast<unsigned long>(rx.decode_fail),
             static_cast<unsigned long>(rx.arm_fail));

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
