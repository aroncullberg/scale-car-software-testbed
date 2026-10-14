#include "motor.h"

#include <algorithm>
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_log.h"

namespace motor {

Motor* volatile Motor::registry_[Motor::MAX_MOTORS] = {};
uint8_t Motor::registry_count_ = 0;
TaskHandle_t Motor::frame_task_handle_ = nullptr;
volatile uint32_t Motor::alarm_count_ = 0;
volatile uint32_t Motor::tick_count_ = 0;
volatile uint32_t Motor::max_gap_us_ = 0;
gptimer_handle_t Motor::frame_timer_ = nullptr;

Motor::Motor(const MotorConfig& config)
    : config_(config)
    , driver_(config.gpio, config.inverted, config.telemetry) {}

Motor::~Motor() {
    state_ = State::UNINITIALIZED;
    for (uint8_t i = 0; i < MAX_MOTORS; i++) {
        if (registry_[i] == this) registry_[i] = nullptr;
    }
}

esp_err_t Motor::init() {
    esp_err_t err = driver_.init();
    if (err != ESP_OK) return err;

    if (registry_count_ >= MAX_MOTORS) {
        ESP_LOGE(TAG, "GPIO %d: no registry slot free", config_.gpio);
        return ESP_ERR_NO_MEM;
    }
    registry_[registry_count_++] = this;

    state_ = State::DISARMED;

    err = start_frame_clock();
    if (err != ESP_OK) {
        state_ = State::UNINITIALIZED;
        return err;
    }

    ESP_LOGI(TAG, "GPIO %d: initialized (disarmed)", config_.gpio);
    return ESP_OK;
}

esp_err_t Motor::start_frame_clock() {
    if (frame_timer_) return ESP_OK;

    // Above the vehicle controller and the RC link: a late frame is a dropped
    // frame, and AM32 measures the gap between frames to keep its lock.
    if (xTaskCreatePinnedToCore(frame_task, "dshot", 3072, nullptr, 20,
                                &frame_task_handle_, 1) != pdPASS) {
        ESP_LOGE(TAG, "failed to create frame task");
        return ESP_ERR_NO_MEM;
    }

    const gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 0,
    };
    esp_err_t err = gptimer_new_timer(&timer_cfg, &frame_timer_);
    if (err != ESP_OK) return err;

    const gptimer_event_callbacks_t cbs = {.on_alarm = frame_alarm};
    err = gptimer_register_event_callbacks(frame_timer_, &cbs, nullptr);
    if (err != ESP_OK) return err;

    const gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = FRAME_PERIOD_US,
        .reload_count = 0,
        .flags = {.auto_reload_on_alarm = true},
    };
    err = gptimer_set_alarm_action(frame_timer_, &alarm_cfg);
    if (err != ESP_OK) return err;

    err = gptimer_enable(frame_timer_);
    if (err != ESP_OK) return err;
    err = gptimer_start(frame_timer_);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "frame clock at %lu us (%lu Hz)",
             (unsigned long)FRAME_PERIOD_US,
             (unsigned long)(1000000 / FRAME_PERIOD_US));
    return ESP_OK;
}

bool IRAM_ATTR Motor::frame_alarm(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*) {
    alarm_count_++;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(frame_task_handle_, &woken);
    return woken == pdTRUE;
}

[[noreturn]] void Motor::frame_task(void*) {
    uint32_t last_tick_us = static_cast<uint32_t>(esp_timer_get_time());
    while (true) {
        // Clears to zero rather than decrementing, so a tick missed while we
        // were busy is dropped instead of firing back to back.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
        const uint32_t gap = now - last_tick_us;
        if (tick_count_ && gap > max_gap_us_) max_gap_us_ = gap;
        last_tick_us = now;
        tick_count_++;

        for (uint8_t i = 0; i < MAX_MOTORS; i++) {
            Motor* m = registry_[i];
            if (m) m->tick();
        }
    }
}

void Motor::arm() {
    if (state_ != State::DISARMED) return;
    esc_seen_ = false;
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
    esc_seen_ = false;
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

void Motor::tick() {
    switch (state_) {
        case State::ARMING        : tick_arming();     break;
        case State::ESTABLISHING  : tick_syncing();    break;
        case State::CHANGING_GEAR : tick_syncing();    break;
        case State::READY         : tick_ready();      break;
        case State::COMMANDING    : tick_commanding(); break;
        // Disarmed still transmits. AM32 has to lock onto the signal before
        // its zero-throttle arm count starts, and that lock takes seconds, so
        // a line that goes quiet until the arm switch moves makes arming look
        // broken. DShot 0 is the same frame the arming hold sends.
        case State::DISARMED      : driver_.send_command(DSHOT_NEUTRAL); break;
        default                   :                    break;
    }
}

void Motor::tick_arming() {
    driver_.send_command(DSHOT_NEUTRAL);

    const TickType_t now = xTaskGetTickCount();

    if (!esc_seen_) {
        const bool answered =
            config_.telemetry && driver_.erpm().status != RpmStatus::NoData;
        if (!answered) {
            if ((now - arming_start_tick_) < pdMS_TO_TICKS(ARM_DETECT_TIMEOUT_MS)) {
                return;
            }
            ESP_LOGW(TAG, "GPIO %d: no telemetry, arming blind", config_.gpio);
        }
        esc_seen_ = true;
        arming_start_tick_ = now;
        ESP_LOGI(TAG, "GPIO %d: ESC detected, holding zero", config_.gpio);
        return;
    }

    if ((now - arming_start_tick_) >= pdMS_TO_TICKS(ARM_HOLD_MS)) {
        last_instruction_tick_ = now;
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
