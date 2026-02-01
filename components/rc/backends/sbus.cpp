//
// Created by cullb on 2025-04-26.
//

#include "sbus.h"

using rclink::channel_value_t;
using rclink::ChannelIndex;

static const char *TAG = "SbusDriver";

namespace proto {
SbusDriver::SbusDriver(const Config &cfg) : cfg_(cfg) {
};

SbusDriver::~SbusDriver() {
  stop();
  ESP_LOGI(TAG, "SbusDriver instance destroyed");
}

esp_err_t SbusDriver::start(const rclink::uart_pins_t& pins, uint32_t timeout_ms) {
    ESP_LOGD(TAG, "Starting SBUS on UART%d (RX: %d, TX: %d)", pins.uart_num, pins.rx_pin, pins.tx_pin);

    if (running_) {
        return ESP_ERR_INVALID_STATE;
    }

    cfg_.uart_num = pins.uart_num;
    cfg_.uart_rx_pin = pins.rx_pin;
    cfg_.uart_tx_pin = pins.tx_pin;

    // Suppress verbose UART logging during detection
    esp_log_level_set("uart", ESP_LOG_WARN);

    esp_err_t ret = configureUART();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART configuration failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    uint32_t valid_frames = 0;
    const uint32_t required_frames = 2;
    uart_event_t e;

    ESP_LOGD(TAG, "Detecting SBUS frames for %lu ms...", timeout_ms);

    while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
        if (xQueueReceive(uart_evt_q_, &e, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (e.type == UART_DATA && e.size == FRAME_SIZE) {
                uint8_t buf[FRAME_SIZE];
                int len = uart_read_bytes(cfg_.uart_num, buf, FRAME_SIZE, 0);

                if (len == FRAME_SIZE && buf[0] == START_BYTE && buf[FRAME_SIZE - 1] == END_BYTE) {
                    valid_frames++;
                    ESP_LOGD(TAG, "Valid SBUS frame %lu", valid_frames);

                    if (valid_frames >= required_frames) {
                        ESP_LOGD(TAG, "SBUS protocol detected! Starting main task...");

                        // Start the SBUS task for continuous operation
                        BaseType_t res = xTaskCreatePinnedToCore(
                            taskEntry,
                            "sbus_driver",
                            4096,
                            this,
                            5,
                            &task_,
                            1
                        );

                        running_ = (res == pdPASS);
                        return running_ ? ESP_OK : ESP_FAIL;
                    }
                } else {
                    ESP_LOGD(TAG, "Invalid SBUS frame: start=%02X, end=%02X", buf[0], buf[FRAME_SIZE-1]);
                    uart_flush_input(cfg_.uart_num);
                }
            } else if (e.type == UART_FIFO_OVF || e.type == UART_BUFFER_FULL) {
                ESP_LOGD(TAG, "UART overflow during detection, flushing");
                uart_flush_input(cfg_.uart_num);
                xQueueReset(uart_evt_q_);
            }
        }
    }

    ESP_LOGD(TAG, "SBUS detection timeout (%lu valid frames, needed %lu)", valid_frames, required_frames);

    // Clean up UART if detection failed
    uart_driver_delete(cfg_.uart_num);
    uart_evt_q_ = nullptr;

    return ESP_ERR_TIMEOUT;
}

esp_err_t SbusDriver::stop() {
  if (!running_)
    return ESP_ERR_INVALID_STATE;
  vTaskDelete(task_);
  task_ = nullptr;
  running_ = false;
  return ESP_OK;
}


esp_err_t SbusDriver::configureUART() {
  const uart_config_t ucfg = {
      .baud_rate = cfg_.buad_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_EVEN,
      .stop_bits = UART_STOP_BITS_2,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
      .source_clk = UART_SCLK_APB,
      .flags = {}
  };

  ESP_ERROR_CHECK_WITHOUT_ABORT(uart_param_config(cfg_.uart_num, &ucfg));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      uart_set_pin(cfg_.uart_num, cfg_.uart_tx_pin, cfg_.uart_rx_pin,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK_WITHOUT_ABORT(uart_driver_install(
    cfg_.uart_num,
    512,
    0,
    20,
    &uart_evt_q_,
    0));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      uart_set_rx_full_threshold(cfg_.uart_num, FRAME_SIZE));
  ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_rx_timeout(cfg_.uart_num, 2));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      uart_set_line_inverse(cfg_.uart_num, UART_SIGNAL_RXD_INV));

  return ESP_OK;
}

void SbusDriver::taskEntry(void *arg) {
  auto *self = static_cast<SbusDriver *>(arg);
  self->run();

  // static_cast<SbusDriver *>(arg)->run();
}

void SbusDriver::run() {

  uart_event_t e;

  // TODO: decrease indenting
  while (true) {
    if (xQueueReceive(uart_evt_q_, &e, portMAX_DELAY) != pdTRUE) {
      ESP_LOGE(TAG, "Failed to receive UART event");
      continue;
    }

    if (e.type != UART_DATA) {
      if (e.type == UART_FIFO_OVF || e.type == UART_BUFFER_FULL) {
        ESP_LOGW(TAG, "FIFO overflow, flushing");
        uart_flush_input(cfg_.uart_num);
        xQueueReset(uart_evt_q_);
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

    if (e.size == FRAME_SIZE) {
      uint8_t buf[FRAME_SIZE];
      int len = uart_read_bytes(cfg_.uart_num,
                                buf,
                                FRAME_SIZE,
                                0);
      if (len == FRAME_SIZE && buf[0] == START_BYTE &&
          buf[FRAME_SIZE - 1] == END_BYTE) {
        ESP_LOGD(TAG, "SBUS frame: %02X %02X %02X %02X %02X", buf[0], buf[1],
                 buf[2], buf[3], buf[4]);
        ++total_frames_;
        processFrame(buf, FRAME_SIZE);
      } else {
        ++total_frames_;
        ++lost_frames_;
        ESP_LOGW(TAG, "Bad frame: %02X %02X %02X %02X %02X", buf[0], buf[1],
                 buf[2], buf[3], buf[4]);
        uart_flush_input(cfg_.uart_num);
      }
    } else {
      ++total_frames_;
      ++lost_frames_;
      ESP_LOGW(TAG, "Bad frame size: %d", e.size);
      uart_flush_input(cfg_.uart_num);
    }
  }
}

void SbusDriver::processFrame(const uint8_t *frame, size_t length) {
  if (length != FRAME_SIZE)
    return;

  int byte_idx = 1;
  int bit_idx = 0;

  for (uint8_t ch = 0; ch < 16; ++ch) {
    uint16_t raw = 0;

    for (int bit = 0; bit < 11; ++bit) {
      if (frame[byte_idx] & (1 << bit_idx))
        raw |= (1 << bit);

      bit_idx++;
      if (bit_idx == 8) {
        bit_idx = 0;
        ++byte_idx;
      }
    }

    channel_value_t scaled = rawToScaled(raw);

    // if (ch == 0) printf("%4u: %4u\n", raw, scaled);

    set_valid_data(true);

    rclink::Backend::push(static_cast<ChannelIndex>(ch), raw, scaled);
  }
}


}