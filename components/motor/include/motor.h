#pragma once

#include "dshot_types.h"
#include "dshot_driver.h"

#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "blackboard/blackboard.h"

namespace motor {

struct RpmReading {
    int32_t rpm[4];
    uint8_t status[4];
    uint8_t count;
};

class Motor {
public:
    explicit Motor(const MotorConfig& config);
    ~Motor();

    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    esp_err_t init();
    void arm();
    void disarm();

    // Signed torque: positive drives in the current gear, negative brakes,
    // zero coasts. The ESC cannot blend the two, so one value carries both.
    bool command(int16_t value);
    bool drive(float value) { return command(static_cast<int16_t>(value * HALF_SPAN)); }
    bool brake(float value) { return command(-static_cast<int16_t>(value * HALF_SPAN)); }

    // Only safe at standstill, and the caller has to establish that. Runs the
    // sync sequence, during which commands are ignored and the motor coasts.
    bool set_gear(Gear gear);
    Gear gear() const { return gear_; }

    RpmSample rpm() const;
    bool cmd(Command beep_or_config);

    // Bench diagnostics, passthrough to the driver.
    DshotDriver::RxStats rx_stats() const { return driver_.rx_stats(); }

    // Alarms counts gptimer firings, ticks counts frame_task iterations.
    // They diverge when the task is starved; both stall together when the
    // timer itself stops. max_gap_us is the longest the ESCs went unfed.
    struct ClockStats {
        uint32_t alarms;
        uint32_t ticks;
        uint32_t max_gap_us;
    };
    static ClockStats clock_stats() {
        return {alarm_count_, tick_count_, max_gap_us_};
    }

    State state() const { return state_; }
    bool is_armed() const {
        State s = state_;
        return s == State::ESTABLISHING || s == State::READY
            || s == State::CHANGING_GEAR || s == State::COMMANDING;
    }

private:
    // One clock drives every motor's frames. AM32 will not lock onto a 500 Hz
    // stream, and four independent tasks at the rate it does need would spend
    // more time switching than transmitting.
    void tick();
    static bool frame_alarm(gptimer_handle_t timer,
                            const gptimer_alarm_event_data_t* edata,
                            void* arg);
    [[noreturn]] static void frame_task(void* arg);
    static esp_err_t start_frame_clock();

    void tick_arming();
    void tick_syncing();
    void tick_ready();
    void tick_commanding();

    bool watchdog_expired() const;
    uint16_t base_for(Gear gear) const;
    uint16_t drive_base() const { return base_for(gear_); }
    uint16_t brake_base() const;
    uint16_t frame_for(int16_t value) const;
    void begin_sync(Gear target);

    MotorConfig config_;
    DshotDriver driver_;
    volatile State state_{State::UNINITIALIZED};
    volatile int16_t command_{0};
    volatile Gear gear_{Gear::FORWARD};
    volatile Gear pending_gear_{Gear::FORWARD};
    volatile uint16_t sync_frame_{0};
    volatile bool cmd_pending_{false};
    Command active_cmd_{Command::MOTOR_STOP};
    uint16_t cmd_frame_{0};
    TickType_t arming_start_tick_{0};
    volatile TickType_t last_instruction_tick_{0};
    // Latched once the ESC answers, so the zero-throttle hold is timed from
    // signal lock rather than from when arm() was called.
    bool esc_seen_{false};

    static constexpr uint8_t MAX_MOTORS = 4;
    // The frame task walks every slot and skips the empty ones, so it never
    // reads registry_count_ and a motor registering mid-run cannot be seen
    // half-written. A single aligned pointer store settles the ordering.
    static Motor* volatile registry_[MAX_MOTORS];
    static uint8_t registry_count_;
    static TaskHandle_t frame_task_handle_;
    static volatile uint32_t alarm_count_;
    static volatile uint32_t tick_count_;
    static volatile uint32_t max_gap_us_;
    static gptimer_handle_t frame_timer_;
    static constexpr uint32_t FRAME_PERIOD_US = 500;

    // Held in milliseconds and converted, because these are wall-clock
    // requirements on the ESC rather than frame counts. Expressing them as
    // frames silently shortened every phase by 4x when the frame rate went
    // from 500 Hz to 2 kHz.
    //
    // Each sync phase is also long enough that a single dropped frame cannot
    // desync us. A missed neutral frame leaves the ESC unflipped, which stalls
    // the wheel rather than spinning it the wrong way, but it still has to be
    // rare.
    static constexpr uint32_t SYNC_PHASE_MS = 20;
    static constexpr uint32_t CMD_PRE_IDLE_MS = 50;
    static constexpr uint32_t CMD_SEND_MS = 20;

    static constexpr uint16_t SYNC_PHASE_FRAMES = SYNC_PHASE_MS * 1000 / FRAME_PERIOD_US;
    static constexpr uint16_t CMD_PRE_IDLE_FRAMES = CMD_PRE_IDLE_MS * 1000 / FRAME_PERIOD_US;
    static constexpr uint16_t CMD_SEND_FRAMES = CMD_SEND_MS * 1000 / FRAME_PERIOD_US;
    static constexpr uint32_t AUTO_DISARM_MS = 250;
    static constexpr const char* TAG = "motor";
};

} // namespace motor

namespace blackboard {
template <> struct Topic<motor::RpmReading> { static constexpr TopicId id = TopicId::rpm; };
}
