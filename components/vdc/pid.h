//
// Created by cullb on 2025-03-27.
//

#ifndef PID_H
#define PID_H

#endif //PID_H

#pragma once
#pragma once

#include <stdint.h>

class PIDController {
public:
    PIDController();
    PIDController(float kp, float ki, float kd, float anti_windup_limit = 0.0f);
    ~PIDController() = default;

    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void setAntiWindupLimit(float limit);

    float getKp() const;
    float getKi() const;
    float getKd() const;
    float getAntiWindupLimit() const;

    void enablePID();
    void disablePID();

    void enableLogging();
    void disableLogging();

    // Reset internal state (integral, previous error, time)
    void reset();

    // Compute PID output based on the given error (calculates dt internally)
    float update(float error);

private:
    float kp_{0.0f};
    float ki_{0.0f};
    float kd_{0.0f};
    float anti_windup_limit_{0.0f};

    float integral_{0.0f};
    float previous_error_{0.0f};
    uint64_t previous_time_us_{0}; // Timestamp in microseconds

    bool enabled_{false};
};
