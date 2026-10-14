#include "dshot_driver.h"

#include <esp_check.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_cpu.h>
#include <esp_attr.h>
#include <driver/gpio.h>
#include <hal/gpio_ll.h>
#include <esp_rom_sys.h>

static const char* TAG = "DshotDriver";

namespace motor {

DshotDriver::DshotDriver(gpio_num_t gpio, bool inverted, bool telemetry)
    : gpio_(gpio), inverted_(inverted), telemetry_(telemetry) {}

DshotDriver::~DshotDriver() {
    if (encoder_) {
        rmt_del_encoder(encoder_);
    }
    if (rx_channel_) {
        if (enabled_ && telemetry_) rmt_disable(rx_channel_);
        rmt_del_channel(rx_channel_);
    }
    if (tx_channel_) {
        if (enabled_) rmt_disable(tx_channel_);
        rmt_del_channel(tx_channel_);
    }
    enabled_ = false;
}

esp_err_t DshotDriver::init() {
    constexpr uint32_t baudrate = 600000;
    constexpr uint32_t post_delay_us = 3;

    if (telemetry_) {
        uint32_t telem_baudrate = baudrate * 5 / 4;
        telemetry_bit_len_ticks_ = static_cast<uint16_t>(
            static_cast<float>(DSHOT_RMT_RESOLUTION_HZ) / telem_baudrate);
    }

    dshot_rmt_encoder_config_t encoder_config = {
        .resolution = DSHOT_RMT_RESOLUTION_HZ,
        .baud_rate = baudrate,
        .bidirectional = inverted_,
        .post_delay_us = post_delay_us,
    };
    ESP_RETURN_ON_ERROR(rmt_new_dshot_esc_encoder(&encoder_config, &encoder_),
                        TAG, "encoder init failed");

    if (telemetry_) {
        const rmt_rx_channel_config_t rx_cfg = {
            .gpio_num = gpio_,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = DSHOT_RMT_RESOLUTION_HZ,
            .mem_block_symbols = DSHOT_MAX_BLOCKS,
            .intr_priority = 1,
            .flags = {
                .with_dma = false,
                .io_loop_back = true,
            },
        };
        ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&rx_cfg, &rx_channel_),
                            TAG, "rx channel init failed");

        rmt_rx_event_callbacks_t rx_cbs = {
            .on_recv_done = rx_done_isr,
        };
        ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(rx_channel_, &rx_cbs, this),
                            TAG, "rx callback register failed");

        rx_config_ = {
            .signal_range_min_ns = 150,
            // Idle threshold ends the receive. The longest run inside a GCR
            // response is 3 bits (~4 us), so this only needs to clear that,
            // and a shorter value gets the callback out well before the next
            // frame goes out.
            .signal_range_max_ns = 10000,
        };
    }

    const rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = gpio_,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = DSHOT_RMT_RESOLUTION_HZ,
        .mem_block_symbols = DSHOT_MAX_BLOCKS,
        .trans_queue_depth = 4,
        .intr_priority = 1,
        .flags = {
            .invert_out = inverted_,
            .with_dma = false,
            .io_loop_back = true,
        },
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_cfg, &tx_channel_),
                        TAG, "tx channel init failed");

    if (telemetry_) {
        rmt_tx_event_callbacks_t tx_cbs = {
            .on_trans_done = tx_done_isr,
        };
        ESP_RETURN_ON_ERROR(rmt_tx_register_event_callbacks(tx_channel_, &tx_cbs, this),
                            TAG, "tx callback register failed");
    }

    tx_config_ = {
        .loop_count = 0, // inf(?)
        .flags = {
            .queue_nonblocking = true,
        },
    };

    throttle_ = {
        .throttle = 0,
        .telemetry_req = false,
    };

    // Both channels are supposed to leave the input buffer on (rx via
    // gpio_input_enable, tx via io_loop_back), but the pad reads a constant 0
    // unless it is asserted here after both exist. Without this the RMT RX
    // never sees an edge, not even from our own transmission.
    if (telemetry_) {
        gpio_ll_input_enable(&GPIO, gpio_);
        gpio_pullup_en(gpio_);
    }

    ESP_RETURN_ON_ERROR(rmt_enable(tx_channel_), TAG, "tx enable failed");

    // The RX channel stays enabled for the life of the driver. Cycling
    // rmt_enable/rmt_disable per frame put a rmt_disable on the next TX in a
    // race with a reception that had started but not yet hit its idle
    // threshold; losing that race aborts the receive with no callback.
    if (telemetry_) {
        ESP_RETURN_ON_ERROR(rmt_enable(rx_channel_), TAG, "rx enable failed");
    }

    mode_ = true;
    enabled_ = true;

    ESP_LOGI(TAG, "initialized on GPIO %d (inverted=%d telemetry=%d)",
             gpio_, inverted_, telemetry_);
    return ESP_OK;
}

void DshotDriver::send_throttle(uint16_t value) {
    if (!enabled_) return;
    if (value < THROTTLE_MIN) value = THROTTLE_MIN;
    if (value > THROTTLE_MAX) value = THROTTLE_MAX;
    throttle_.throttle = value;
    throttle_.telemetry_req = false;
    mode_tx();
    const uint32_t t0 = esp_cpu_get_cycle_count();
    if (rmt_transmit(tx_channel_, encoder_, &throttle_, sizeof(throttle_), &tx_config_) != ESP_OK) {
        tx_fail_count_++;
    }
    const uint32_t dt = esp_cpu_get_cycle_count() - t0;
    tx_cyc_last_ = dt;
    if (dt > tx_cyc_max_) tx_cyc_max_ = dt;
}

void DshotDriver::send_command(uint16_t cmd) {
    if (!enabled_ || cmd >= THROTTLE_MIN) return;
    throttle_.throttle = cmd;
    throttle_.telemetry_req = true;
    mode_tx();
    if (rmt_transmit(tx_channel_, encoder_, &throttle_, sizeof(throttle_), &tx_config_) != ESP_OK) {
        tx_fail_count_++;
    }
    throttle_.telemetry_req = false;
}

uint32_t DshotDriver::selftest_loopback(int frames) {
    if (!enabled_ || !telemetry_) return 0;

    const uint32_t before = rx_isr_count_;
    for (int i = 0; i < frames; i++) {
        // reset the receiver in case a previous receive is still pending
        rmt_disable(rx_channel_);
        rmt_enable(rx_channel_);
        rx_pending_ = false;

        // stay push-pull: we want to hear ourselves, not hand the line over
        gpio_ll_od_disable(&GPIO, gpio_);
        mode_ = true;

        if (rmt_receive(rx_channel_, rx_buf_, sizeof(rx_buf_), &rx_config_) != ESP_OK) {
            arm_fail_count_++;
            continue;
        }
        rx_pending_ = true;
        throttle_.throttle = 0;
        throttle_.telemetry_req = true;
        rmt_transmit(tx_channel_, encoder_, &throttle_, sizeof(throttle_), &tx_config_);

        // Read the pad directly while our own frame goes out. If this never
        // changes, the GPIO input itself is dead and no RMT routing fix helps.
        int last = gpio_ll_get_level(&GPIO, gpio_);
        int lo = (last == 0), hi = (last != 0);
        for (int k = 0; k < 4000; k++) {
            const int now = gpio_ll_get_level(&GPIO, gpio_);
            if (now != last) { pad_edges_++; last = now; }
            if (now) hi++; else lo++;
        }
        pad_high_ = hi;
        pad_low_ = lo;

        esp_rom_delay_us(500);
    }
    return rx_isr_count_ - before;
}

ErpmSample DshotDriver::erpm() const {
    if (!telemetry_) return {0, RpmStatus::NoData};

    uint32_t age = static_cast<uint32_t>(esp_timer_get_time()) - telemetry_stamp_us_;
    if (age > TELEMETRY_TIMEOUT_US) return {0, RpmStatus::NoData};

    uint32_t data = gcr_to_erpm_data(telemetry_gcr_);
    uint32_t value = erpm_data_to_erpm_period(data);

    if (value == INVALID_TELEMETRY) return {0, RpmStatus::NoData};
    if (value == 0) return {0, RpmStatus::Stopped};
    return {value, RpmStatus::Valid};
}

bool IRAM_ATTR DshotDriver::tx_done_isr(rmt_channel_handle_t channel,
                                          const rmt_tx_done_event_data_t* edata,
                                          void* user_data) {
    auto* self = static_cast<DshotDriver*>(user_data);
    self->tx_done_count_++;
    self->mode_rx();
    return false;
}

bool IRAM_ATTR DshotDriver::rx_done_isr(rmt_channel_handle_t channel,
                                          const rmt_rx_done_event_data_t* edata,
                                          void* user_data) {
    auto* self = static_cast<DshotDriver*>(user_data);
    self->rx_pending_ = false;
    self->rx_isr_count_++;
    self->rx_symbol_count_ = edata->num_symbols;

    // Handing the line to the ESC puts an edge on the pad, which starts a
    // reception that then idles out empty. One of these lands between every
    // pair of real replies, so letting it through overwrites good telemetry
    // with a failed decode half the time.
    if (edata->num_symbols == 0 || edata->received_symbols[0].duration0 == 0) {
        self->empty_rx_count_++;
        return false;
    }

    self->telemetry_gcr_ = extract_telemetry_gcr(
        edata->received_symbols, edata->num_symbols, self->telemetry_bit_len_ticks_);
    if (gcr_to_erpm_data(self->telemetry_gcr_) == INVALID_TELEMETRY) {
        self->decode_fail_count_++;
        const size_t n = edata->num_symbols;
        self->fail_d_[0] = n > 0 ? edata->received_symbols[0].duration0 : 0;
        self->fail_d_[1] = n > 0 ? edata->received_symbols[0].duration1 : 0;
        self->fail_d_[2] = n > 1 ? edata->received_symbols[1].duration0 : 0;
        self->fail_d_[3] = n > 1 ? edata->received_symbols[1].duration1 : 0;
    }
    self->telemetry_stamp_us_ = static_cast<uint32_t>(esp_timer_get_time());
    return false;
}

void IRAM_ATTR DshotDriver::mode_tx() {
    if (!mode_) {
        disable_rx();
        mode_ = true;
    }
}

void IRAM_ATTR DshotDriver::mode_rx() {
    if (mode_) {
        const uint32_t t0 = esp_cpu_get_cycle_count();
        enable_rx();
        mode_ = false;
        // RMT RX only completes on its idle threshold once it has captured at
        // least one edge. A silent ESC therefore leaves the receiver in RUN
        // forever and every later rmt_receive is rejected, so the stale one
        // has to be aborted first. Conditional because the reset also masks
        // the channel's interrupt, and once the ESC answers it never applies.
        if (rx_pending_) {
            rmt_disable(rx_channel_);
            rmt_enable(rx_channel_);
            rx_pending_ = false;
            stale_reset_count_++;
        }
        if (rmt_receive(rx_channel_, rx_buf_, sizeof(rx_buf_), &rx_config_) == ESP_OK) {
            rx_pending_ = true;
            rx_armed_count_++;
        } else {
            arm_fail_count_++;
        }
        const uint32_t dt = esp_cpu_get_cycle_count() - t0;
        arm_cyc_last_ = dt;
        if (dt > arm_cyc_max_) arm_cyc_max_ = dt;
    }
}

void IRAM_ATTR DshotDriver::enable_rx() {
    // Hand the line to the ESC: open drain so our idle-high is released.
    gpio_ll_od_enable(&GPIO, gpio_);
}

void IRAM_ATTR DshotDriver::disable_rx() {
    gpio_ll_od_disable(&GPIO, gpio_);
}

static uint32_t IRAM_ATTR duration_to_bit_len(uint32_t duration, uint32_t len) {
    return (duration + (len >> 1)) / len;
}

static uint32_t IRAM_ATTR push_bits(uint32_t value, uint32_t bit_val, size_t bit_len) {
    while (bit_len--) {
        value <<= 1;
        value |= bit_val;
    }
    return value;
}

uint32_t IRAM_ATTR DshotDriver::extract_telemetry_gcr(rmt_symbol_word_t* symbols,
                                                        size_t num,
                                                        uint32_t bit_len_ticks) {
    if (symbols->level0 != 0) return 0;

    int bit_count = 0;
    uint32_t value = 0;
    for (size_t i = 0; i < num; i++) {
        if (!symbols[i].duration0) break;
        uint32_t len0 = duration_to_bit_len(symbols[i].duration0, bit_len_ticks);
        if (len0) {
            value = push_bits(value, symbols[i].level0, len0);
            bit_count += len0;
        }
        if (!symbols[i].duration1) break;
        uint32_t len1 = duration_to_bit_len(symbols[i].duration1, bit_len_ticks);
        if (len1) {
            value = push_bits(value, symbols[i].level1, len1);
            bit_count += len1;
        }
    }

    if (bit_count < 21) {
        value = push_bits(value, 0x1, 21 - bit_count);
    }

    value &= 0xfffff;
    value = value ^ (value >> 1);
    return value;
}

uint32_t DshotDriver::gcr_to_erpm_data(uint32_t value) {
    if (!value) return INVALID_TELEMETRY;

    static const unsigned char GCR_decode[32] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 9, 10, 11,
        0xFF, 13, 14, 15,
        0xFF, 0xFF, 2, 3,
        0xFF, 5, 6, 7,
        0xFF, 0, 8, 1,
        0xFF, 4, 12, 0xFF,
    };

    uint32_t decoded = GCR_decode[value & 0x1f];
    decoded |= GCR_decode[(value >> 5) & 0x1f] << 4;
    decoded |= GCR_decode[(value >> 10) & 0x1f] << 8;
    decoded |= GCR_decode[(value >> 15) & 0x1f] << 12;

    uint32_t csum = decoded;
    csum = csum ^ (csum >> 8);
    csum = csum ^ (csum >> 4);

    if ((csum & 0xf) != 0xf || decoded > 0xffff) {
        return INVALID_TELEMETRY;
    }

    return decoded >> 4;
}

uint32_t DshotDriver::erpm_data_to_erpm_period(uint32_t value) {
    if (!value || value == INVALID_TELEMETRY) return INVALID_TELEMETRY;
    if (value == 0x0fff) return 0;

    value = (value & 0x01ff) << ((value & 0xfe00) >> 9);

    if (!value || value == INVALID_TELEMETRY) return INVALID_TELEMETRY;

    return (1000000 * 60 / 100 + value / 2) / value;
}

} // namespace motor
