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

    // Bench self-test: arms RX and then transmits with the line still driven
    // push-pull, so the receiver should capture our own frame. Distinguishes
    // "RMT RX cannot see this pin at all" from "we miss the ESC's reply".
    // Returns the number of receive completions out of `frames` attempts.
    uint32_t selftest_loopback(int frames);

    // Populated by selftest_loopback: raw pad reads taken while we transmit.
    struct PadProbe { uint32_t edges; uint32_t high; uint32_t low; };
    PadProbe pad_probe() const { return {pad_edges_, pad_high_, pad_low_}; }

    // Bench diagnostics for the half-duplex path. Nothing depends on these.
    struct RxStats {
        uint32_t rx_isr;      // RX completions seen
        uint32_t rx_symbols;  // symbols in the last completion
        uint32_t decode_fail; // completions whose GCR/CRC decode failed
        uint32_t arm_fail;    // rmt_enable or rmt_receive returned an error
        uint32_t tx_done;     // TX completion callbacks
        uint32_t rx_armed;    // times rmt_receive was actually issued
        uint32_t tx_fail;      // rmt_transmit returned an error
        uint32_t stale_reset;  // receptions aborted because the ESC never answered
        uint32_t empty_rx;     // completions holding nothing, from our own handover
        uint32_t arm_cyc_last; // CPU cycles spent in mode_rx() arming the receiver
        uint32_t arm_cyc_max;
        uint32_t tx_cyc_last;  // CPU cycles spent inside rmt_transmit()
        uint32_t tx_cyc_max;
        // First two symbol durations of the last reception that failed to
        // decode, in RMT ticks. A handover glitch is one short symbol; a real
        // GCR reply is several at the telemetry bit period, and these give
        // that period directly when the decode is wrong about it.
        uint16_t fail_d0;
        uint16_t fail_d1;
        uint16_t fail_d2;
        uint16_t fail_d3;
    };
    RxStats rx_stats() const {
        return {rx_isr_count_, rx_symbol_count_, decode_fail_count_,
                arm_fail_count_, tx_done_count_, rx_armed_count_, tx_fail_count_,
                stale_reset_count_, empty_rx_count_, arm_cyc_last_, arm_cyc_max_,
                tx_cyc_last_, tx_cyc_max_,
                fail_d_[0], fail_d_[1], fail_d_[2], fail_d_[3]};
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
    // Written by tx_done_isr and read by the task in mode_tx(); must not be
    // cached in a register or the half-duplex handoff stalls.
    volatile bool mode_{true}; // true = TX

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
    volatile uint32_t tx_done_count_{0};
    volatile uint32_t rx_armed_count_{0};
    volatile uint32_t tx_fail_count_{0};
    volatile uint32_t stale_reset_count_{0};
    volatile uint32_t empty_rx_count_{0};
    // Set when rmt_receive is armed, cleared by the recv-done ISR. Both writers
    // are the same RMT interrupt, so no lock is needed.
    volatile bool rx_pending_{false};
    volatile uint32_t arm_cyc_last_{0};
    volatile uint32_t arm_cyc_max_{0};
    volatile uint32_t tx_cyc_last_{0};
    volatile uint32_t tx_cyc_max_{0};
    volatile uint16_t fail_d_[4]{};
    volatile uint32_t pad_edges_{0};
    volatile uint32_t pad_high_{0};
    volatile uint32_t pad_low_{0};

    static constexpr uint32_t INVALID_TELEMETRY = 0xffff;
    static constexpr uint32_t TELEMETRY_TIMEOUT_US = 10000;
};

} // namespace motor
