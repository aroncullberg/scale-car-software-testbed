#pragma once

#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>

#include "dshot_rmt_encoder.h"
#include "dshot_types.h"

#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_RMT_RESOLUTION_HZ 32000000
#else
#define DSHOT_RMT_RESOLUTION_HZ 40000000
#endif

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define DSHOT_MAX_BLOCKS 64
#else
#define DSHOT_MAX_BLOCKS 48
#endif

namespace motor {

class DshotDriver {
public:
    DshotDriver(gpio_num_t gpio, bool inverted, bool telemetry);
    ~DshotDriver();

    DshotDriver(const DshotDriver&) = delete;
    DshotDriver& operator=(const DshotDriver&) = delete;

    esp_err_t init();

    void send_throttle(uint16_t value);
    void send_command(uint16_t cmd);

    ErpmSample erpm() const;

    // Bench diagnostics for the half-duplex path. Nothing depends on these.
    struct RxStats {
        uint32_t rx_isr;      // RX completions seen
        uint32_t rx_symbols;  // symbols in the last completion
        uint32_t decode_fail; // completions whose GCR/CRC decode failed
        uint32_t arm_fail;    // rmt_enable or rmt_receive returned an error
    };
    RxStats rx_stats() const {
        return {rx_isr_count_, rx_symbol_count_, decode_fail_count_, arm_fail_count_};
    }

    static float erpm_to_rpm_ratio(uint8_t poles) {
        constexpr float ERPM_PER_LSB = 100.0f;
        return ERPM_PER_LSB / (poles / 2.0f);
    }

private:
    void mode_tx();
    void mode_rx();
    void enable_rx();
    void disable_rx();

    static bool rx_done_isr(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t* edata, void* user_data);
    static bool tx_done_isr(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t* edata, void* user_data);

    static uint32_t extract_telemetry_gcr(rmt_symbol_word_t* symbols, size_t num, uint32_t bit_len_ticks);
    static uint32_t gcr_to_erpm_data(uint32_t value);
    static uint32_t erpm_data_to_erpm_period(uint32_t value);

    gpio_num_t gpio_;
    bool inverted_;
    bool telemetry_;
    bool enabled_{false};
    bool mode_{true}; // true = TX

    rmt_channel_handle_t rx_channel_{nullptr};
    rmt_channel_handle_t tx_channel_{nullptr};
    rmt_encoder_handle_t encoder_{nullptr};
    rmt_receive_config_t rx_config_{};
    rmt_transmit_config_t tx_config_{};
    dshot_rmt_throttle_t throttle_{};
    rmt_symbol_word_t rx_buf_[DSHOT_MAX_BLOCKS]{};

    uint16_t telemetry_bit_len_ticks_{0};
    volatile uint32_t telemetry_gcr_{0};

    // Low 32 bits of esp_timer_get_time() at the last RX completion. Single word
    // so the ISR write cannot tear, and unsigned subtraction handles the ~71 min
    // wrap. Without it a silent ESC leaves the last decode looking fresh forever.
    volatile uint32_t telemetry_stamp_us_{0};

    volatile uint32_t rx_isr_count_{0};
    volatile uint32_t rx_symbol_count_{0};
    volatile uint32_t decode_fail_count_{0};
    volatile uint32_t arm_fail_count_{0};

    static constexpr uint32_t INVALID_TELEMETRY = 0xffff;
    static constexpr uint32_t TELEMETRY_TIMEOUT_US = 10000;
};

} // namespace motor
