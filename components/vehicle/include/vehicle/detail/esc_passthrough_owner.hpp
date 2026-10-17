#pragma once

#include <array>
#include <cstdint>
#include "dshot/driver.hpp"
#include "dshot/link.hpp"
#include "dshot/passthrough.hpp"
#include "esc_passthrough.h"
#include "esp_log.h"

namespace vehicle::detail {

// Parser calls, callbacks and motor setters all run on the vehicle task.
class EscPassthroughOwner {
public:
    // Starting interval from the component's DShot example; verify on the ESC.
    static constexpr std::int64_t rearm_us = 5000000;

    EscPassthroughOwner(dshot::Driver& driver, std::array<dshot::Link*, 4> links,
                        std::int64_t (*clock_us)())
        : driver_(driver), links_(links), handoff_(driver, links.data(), links.size()),
          clock_us_(clock_us) {}

    esc_passthrough_lifecycle_t callbacks() {
        return {this, acquire_callback, release_callback};
    }

    bool owns_motor_pins() const { return handoff_.held(); }
    bool motor_fault_latched() const { return fault_latched_; }
    bool rearming() const { return rearming_; }
    std::int64_t rearm_deadline_us() const { return ready_at_us_; }
    void finish_rearming() { rearming_ = false; }

    void close_motor_links_after_fault() {
        fault_latched_ = true;
        rearming_ = false;
        driver_.stop();
        for (auto* link : links_) link->end();
    }

private:
    static esp_err_t acquire_callback(void* ctx) {
        auto& owner = *static_cast<EscPassthroughOwner*>(ctx);
        owner.rearming_ = false;
        owner.saved_log_level_ = esp_log_get_level_master();
        esp_log_set_level_master(ESP_LOG_NONE);
        const auto error = owner.handoff_.acquire();
        if (error != ESP_OK) {
            owner.close_motor_links_after_fault();
            esp_log_set_level_master(owner.saved_log_level_);
        }
        return error;
    }

    static esp_err_t release_callback(void* ctx, const esc_session_end_t* result) {
        auto& owner = *static_cast<EscPassthroughOwner*>(ctx);
        const bool resume = result->reason == ESC_SESSION_NORMAL_EXIT &&
                            result->io_released && !result->needs_reboot_mask &&
                            !owner.fault_latched_;
        const auto error = owner.handoff_.release(resume);
        if (error != ESP_OK || !resume) owner.close_motor_links_after_fault();
        else {
            owner.ready_at_us_ = owner.clock_us_() + rearm_us;
            owner.rearming_ = true;
        }
        esp_log_set_level_master(owner.saved_log_level_);
        return error;
    }

    dshot::Driver& driver_;
    std::array<dshot::Link*, 4> links_;
    dshot::Passthrough handoff_;
    std::int64_t (*clock_us_)();
    std::int64_t ready_at_us_ = 0;
    esp_log_level_t saved_log_level_ = ESP_LOG_INFO;
    bool rearming_ = false;
    bool fault_latched_ = false;
};

} // namespace vehicle::detail
