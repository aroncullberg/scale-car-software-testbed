#include "motor.h"

#include <algorithm>
#include "esp_log.h"

namespace motor {

Motor::Motor(const MotorConfig& config)
    : config_(config)
    , driver_(config.gpio, config.inverted, config.telemetry) {}

Motor::~Motor() {
    if (task_handle_) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }
}

esp_err_t Motor::init() {
    esp_err_t err = driver_.init();
    if (err != ESP_OK) return err;

    state_ = State::DISARMED;

    BaseType_t result = xTaskCreatePinnedToCore(
        task_entry, "motor", 4096, this, 5, &task_handle_, 1);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "failed to create task for GPIO %d", config_.gpio);
        state_ = State::UNINITIALIZED;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "GPIO %d: initialized (disarmed)", config_.gpio);
    return ESP_OK;
}

void Motor::arm() {
    if (state_ != State::DISARMED) return;
    last_instruction_tick_ = xTaskGetTickCount();
    arming_start_tick_ = last_instruction_tick_;
    command_ = 0;
    state_ = State::ARMING;
    ESP_LOGI(TAG, "GPIO %d: arming started", config_.gpio);
}

void Motor::disarm() {
    state_ = State::DISARMED;
    command_ = 0;
    cmd_pending_ = false;
    gear_ = Gear::FORWARD;
}

uint16_t Motor::base_for(Gear gear) const {
    const bool high = (config_.mounting == Mounting::NORMAL) == (gear == Gear::FORWARD);
    return high ? HALF_HIGH_BASE : HALF_LOW_BASE;
}

uint16_t Motor::brake_base() const {
    return drive_base() == HALF_HIGH_BASE ? HALF_LOW_BASE : HALF_HIGH_BASE;
}

uint16_t Motor::frame_for(int16_t value) const {
    if (value == 0) return drive_base();
    if (value > 0) return drive_base() + std::min<int16_t>(value, HALF_SPAN);
    return brake_base() + std::min<int16_t>(-value, HALF_SPAN);
}

bool Motor::command(int16_t value) {
    if (!is_armed()) return false;

    last_instruction_tick_ = xTaskGetTickCount();

    if (state_ != State::READY) return false;
    command_ = std::clamp<int16_t>(value, -HALF_SPAN, HALF_SPAN);
    return true;
}

bool Motor::set_gear(Gear gear) {
    if (state_ != State::READY) return false;
    if (gear == gear_) return true;
    begin_sync(gear);
    state_ = State::CHANGING_GEAR;
    ESP_LOGI(TAG, "GPIO %d: gear -> %s", config_.gpio,
             gear == Gear::FORWARD ? "FORWARD" : "REVERSE");
    return true;
}

bool Motor::cmd(Command beep_or_config) {
    if (state_ != State::READY || command_ != 0) return false;
    last_instruction_tick_ = xTaskGetTickCount();
    active_cmd_ = beep_or_config;
    cmd_pending_ = true;
    return true;
}

RpmSample Motor::rpm() const {
    ErpmSample e = driver_.erpm();
    if (e.status != RpmStatus::Valid) return {0, e.status};
    return {static_cast<int32_t>(e.erpm * DshotDriver::erpm_to_rpm_ratio(config_.poles)),
            RpmStatus::Valid};
}

void Motor::begin_sync(Gear target) {
    pending_gear_ = target;
    sync_frame_ = 0;
    command_ = 0;
}

bool Motor::watchdog_expired() const {
    return (xTaskGetTickCount() - last_instruction_tick_) >= pdMS_TO_TICKS(AUTO_DISARM_MS);
}

void Motor::task_entry(void* arg) {
    static_cast<Motor*>(arg)->task_loop();
}

[[noreturn]] void Motor::task_loop() {
    TickType_t last_wake = xTaskGetTickCount();
    constexpr TickType_t period = pdMS_TO_TICKS(2);

    while (true) {
        switch (state_) {
            case State::ARMING        : tick_arming();     break;
            case State::ESTABLISHING  : tick_syncing();    break;
            case State::CHANGING_GEAR : tick_syncing();    break;
            case State::READY         : tick_ready();      break;
            case State::COMMANDING    : tick_commanding(); break;
            default                   :                    break;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

void Motor::tick_arming() {
    driver_.send_command(DSHOT_NEUTRAL);

    if ((xTaskGetTickCount() - arming_start_tick_) >= pdMS_TO_TICKS(ARM_DELAY_MS)) {
        last_instruction_tick_ = xTaskGetTickCount();
        begin_sync(gear_);
        state_ = State::ESTABLISHING;
        ESP_LOGI(TAG, "GPIO %d: armed", config_.gpio);
    }
}

// Three phases against AM32's return_to_center latch. Phase one commands the
// target half, which brakes at near-zero duty if the ESC is facing the other
// way. Phase two drops below 48, the only thing that arms the latch. Phase
// three commands the target half again, consuming it and flipping the ESC.
// A wheel already facing the target way passes through untouched.
void Motor::tick_syncing() {
    if (watchdog_expired()) {
        ESP_LOGW(TAG, "GPIO %d: auto-disarm (watchdog)", config_.gpio);
        disarm();
        return;
    }

    const uint16_t target = base_for(pending_gear_);
    sync_frame_++;

    if (sync_frame_ <= SYNC_PHASE_FRAMES) {
        driver_.send_throttle(target);
    } else if (sync_frame_ <= 2 * SYNC_PHASE_FRAMES) {
        driver_.send_command(DSHOT_NEUTRAL);
    } else if (sync_frame_ <= 3 * SYNC_PHASE_FRAMES) {
        driver_.send_throttle(target);
    } else {
        gear_ = pending_gear_;
        command_ = 0;
        last_instruction_tick_ = xTaskGetTickCount();
        state_ = State::READY;
    }
}

void Motor::tick_ready() {
    if (watchdog_expired()) {
        ESP_LOGW(TAG, "GPIO %d: auto-disarm (watchdog)", config_.gpio);
        disarm();
        return;
    }

    if (cmd_pending_) {
        cmd_pending_ = false;
        cmd_frame_ = 0;
        command_ = 0;
        state_ = State::COMMANDING;
        return;
    }

    driver_.send_throttle(frame_for(command_));
}

// AM32 only processes commands with the motor stopped, and the command frames
// themselves force its input to zero, so this coasts first rather than braking.
void Motor::tick_commanding() {
    cmd_frame_++;

    if (cmd_frame_ <= CMD_PRE_IDLE_FRAMES) {
        driver_.send_throttle(drive_base());
    } else if (cmd_frame_ <= CMD_PRE_IDLE_FRAMES + CMD_SEND_FRAMES) {
        driver_.send_command(static_cast<uint16_t>(active_cmd_));
    } else {
        command_ = 0;
        last_instruction_tick_ = xTaskGetTickCount();
        state_ = State::READY;
    }
}

} // namespace motor
