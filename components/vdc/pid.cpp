//
// Created by cullb on 2025-03-27.
//

#include "pid.h"

#include <algorithm>
#include <esp_timer.h>

PIDController::PIDController() {
    kp_ = 0.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;
    anti_windup_limit_ = 0.0f;
    reset();
}

PIDController::PIDController(float kp, float ki, float kd, float anti_windup_limit) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    anti_windup_limit_ = anti_windup_limit;
    reset();
}

void PIDController::setKp(float kp) {
    kp_ = std::max(kp, 0.0f);
}

void PIDController::setKi(float ki) {
    ki_ = std::max(ki, 0.0f);
}

void PIDController::setKd(float kd) {
    kd_ = std::max(kd, 0.0f);
}

void PIDController::setAntiWindupLimit(float limit) {
    anti_windup_limit_ = std::max(limit, 0.0f);
}

float PIDController::getKp() const {
    return kp_;
}
float PIDController::getKi() const {
    return ki_;
}
float PIDController::getKd() const {
    return kd_;
}
float PIDController::getAntiWindupLimit() const {
    return anti_windup_limit_;
}

void PIDController::enablePID() {
    enabled_ = true;
}
void PIDController::disablePID() {
    enabled_ = false;
}

void PIDController::reset() {
    integral_ = 0.0f;
    previous_error_ = 0.0f;
    previous_time_us_ = esp_timer_get_time();
}

float PIDController::update(float error) {
    uint64_t current_time_us = esp_timer_get_time();
    float dt = (current_time_us - previous_time_us_) / 1000000.0f; // μs to s

    // NOTE: safety thing, many time not many good
    if (dt > 0.1f) dt = 0.02f;

    previous_time_us_ = current_time_us;

    float p_term = error * kp_;

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -anti_windup_limit_, anti_windup_limit_);
    float i_term = integral_ * ki_;

    float error_rate = (error - previous_error_) / dt;
    previous_error_ = error;
    float d_term = error_rate * kd_;

    return p_term + i_term + d_term;
}



