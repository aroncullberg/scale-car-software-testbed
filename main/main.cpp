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
#include "dshot_driver.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "nmea_driver.h"
#include "pin_test.h"
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

namespace {

constexpr gpio_num_t GPIO_FL = GPIO_NUM_18;
constexpr gpio_num_t GPIO_FR = GPIO_NUM_39;
constexpr gpio_num_t GPIO_RL = GPIO_NUM_38;
constexpr gpio_num_t GPIO_RR = GPIO_NUM_40;

// Reads each pad with the internal pull-up, then the pull-down, and reports
// whether the input follows. Standalone: no RMT, no driver, nothing else
// configured on these pins yet. A pad that reads STUCK cannot receive
// telemetry no matter what the ESC does.

void probe_pads() {
  struct Pad { gpio_num_t gpio; const char *note; };

  static const Pad pads[] = {
      {GPIO_FL, "motor_fl"},
      {GPIO_FR, "motor_fr"},
      {GPIO_RL, "motor_rl"},
      {GPIO_RR, "motor_rr"},
  };

  for (const Pad &p : pads) {
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << p.gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    esp_rom_delay_us(3000);
    const int up = gpio_get_level(p.gpio);

    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&cfg);
    esp_rom_delay_us(3000);
    const int down = gpio_get_level(p.gpio);

    const char *verdict = (up == 1 && down == 0) ? "READABLE"
                          : (up == down)         ? "STUCK"
                                                 : "inconclusive";
    ESP_LOGI(TAG, "  GPIO%-2d pullup=%d pulldown=%d  %-12s (%s)",
             static_cast<int>(p.gpio), up, down, verdict, p.note);
  }
}

} // namespace

extern "C" void app_main(void) {
  print_reset_reason();

  ESP_LOGI(TAG, "Car main application starting");

  // Whole-chip pad walk. Its job was finding the dead pad on the old module
  // and it is done; probe_pads() below still checks the ESC pins.
  // pin_test::run_all();

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

  Servo::Config servo_cfg = {
      .gpio_num = GPIO_NUM_47,
      .min_pulse_width_us = 500,
      .max_pulse_width_us = 2500,
      .freq_hz = 50,
      .failsafe_position = 1000 // Center position (its stupid i know but
                                // this is rc cahnle , other is us)
  };

  static Servo steering_servo(servo_cfg);

  Servo::Config aero_cfg = {.gpio_num = GPIO_NUM_4,
                            .min_pulse_width_us = 1250,
                            .max_pulse_width_us = 2300,
                            .freq_hz = 50,
                            .failsafe_position = 1000};

  static Servo aero_servo(aero_cfg);
  aero_servo.setRange(100);
  aero_servo.setInvert(false);

  Servo::Config aux1_cfg = {.gpio_num = GPIO_NUM_6,
                            .min_pulse_width_us = 700,
                            .max_pulse_width_us = 2500,
                            .freq_hz = 50,
                            .failsafe_position = 1000,
                            .group_id = 1};
  static Servo aux_servo1(aux1_cfg);
  aux_servo1.setInvert(false);
  aux_servo1.setRange(100);

  Servo::Config aux2_cfg = {.gpio_num = GPIO_NUM_7,
                            .min_pulse_width_us = 500,
                            .max_pulse_width_us = 2300,
                            .freq_hz = 50,
                            .failsafe_position = 1000,
                            .group_id = 1};
  static Servo aux_servo2(aux2_cfg);
  aux_servo2.setInvert(true);
  aux_servo2.setRange(100);

  static motor::Motor motor_fl(
      {.gpio = GPIO_FL, .mounting = motor::Mounting::NORMAL});
  static motor::Motor motor_fr(
      {.gpio = GPIO_FR, .mounting = motor::Mounting::NORMAL});
  static motor::Motor motor_rl(
      {.gpio = GPIO_RL, .mounting = motor::Mounting::MIRRORED});
  static motor::Motor motor_rr(
      {.gpio = GPIO_RR, .mounting = motor::Mounting::NORMAL});

  probe_pads();

  motor::Motor *motors[] = {&motor_fl, &motor_fr, &motor_rl, &motor_rr};
  for (motor::Motor *m : motors) {
    result = m->init();
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "motor init failed: %s", esp_err_to_name(result));
      return;
    }
  }

  vehicle::Controller::Config vehicle_cfg = {
      .steering_servo = steering_servo,
      .aero_servo = aero_servo,
      .aux_servo1 = aux_servo1,
      .aux_servo2 = aux_servo2,
      .motor_fl = motor_fl,
      .motor_fr = motor_fr,
      .motor_rl = motor_rl,
      .motor_rr = motor_rr,
  };

  static vehicle::Controller vehicle_controller(vehicle_cfg);

  result = vehicle_controller.start();
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Vehicle controller start failed: %s",
             esp_err_to_name(result));
    return;
  }

  ESP_LOGI(TAG, "vehicle controller started");

  struct Wheel {
    const char *name;
    motor::Motor *m;
  };
  const Wheel wheels[] = {{"fl", &motor_fl},
                          {"fr", &motor_fr},
                          {"rl", &motor_rl},
                          {"rr", &motor_rr}};

  // Counters move every frame, so printing them on change would print forever.
  // State and telemetry status are what matter moment to moment: those print
  // the instant they change, the counters only on a slow heartbeat.
  uint32_t last_key = 0xFFFFFFFF;
  int heartbeat = 0;

  while (true) {
    uint32_t key = 0;
    for (const Wheel &w : wheels) {
      key = (key << 8) | (static_cast<uint32_t>(w.m->state()) << 4) |
            static_cast<uint32_t>(w.m->rpm().status);
    }

    if (key != last_key || --heartbeat <= 0) {
      last_key = key;
      heartbeat = 20;
      char line[384];
      size_t off = 0;
      for (const Wheel &w : wheels) {
        const motor::RpmSample r = w.m->rpm();
        const motor::DshotDriver::RxStats rx = w.m->rx_stats();
        off += snprintf(line + off, sizeof(line) - off,
                        "%s s%d/%d rpm%ld rx%lu bad%lu mt%lu stl%lu  ", w.name,
                        static_cast<int>(w.m->state()),
                        static_cast<int>(r.status), (long)r.rpm,
                        (unsigned long)rx.rx_isr, (unsigned long)rx.decode_fail,
                        (unsigned long)rx.empty_rx,
                        (unsigned long)rx.stale_reset);
        if (off >= sizeof(line)) break;
      }
      const motor::Motor::ClockStats ck = motor::Motor::clock_stats();
      ESP_LOGI(TAG, "clock alarms=%lu ticks=%lu missed=%ld maxgap=%luus",
               (unsigned long)ck.alarms, (unsigned long)ck.ticks,
               (long)(ck.alarms - ck.ticks), (unsigned long)ck.max_gap_us);
      ESP_LOGI(TAG, "%s", line);
    }

    vTaskDelay(pdMS_TO_TICKS(250));
  }
}
