#pragma once

#include "dshot_types.h"
#include "dshot_driver.h"

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

    State state() const { return state_; }
    bool is_armed() const {
        State s = state_;
        return s == State::ESTABLISHING || s == State::READY
            || s == State::CHANGING_GEAR || s == State::COMMANDING;
    }

private:
    static void task_entry(void* arg);
    [[noreturn]] void task_loop();

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
    TaskHandle_t task_handle_{nullptr};

    // Each sync phase is repeated so a single dropped frame cannot desync us.
    // A missed neutral frame leaves the ESC unflipped, which stalls the wheel
    // rather than spinning it the wrong way, but it still has to be rare.
    static constexpr uint16_t SYNC_PHASE_FRAMES = 10;
    static constexpr uint16_t CMD_PRE_IDLE_FRAMES = 25;
    static constexpr uint16_t CMD_SEND_FRAMES = 10;
    static constexpr uint32_t AUTO_DISARM_MS = 250;
    static constexpr const char* TAG = "motor";
};

} // namespace motor

namespace blackboard {
template <> struct Topic<motor::RpmReading> { static constexpr TopicId id = TopicId::rpm; };
}
