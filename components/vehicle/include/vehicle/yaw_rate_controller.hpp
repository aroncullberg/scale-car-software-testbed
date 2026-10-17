#pragma once

#include <cstdint>

namespace vehicle {

struct YawRateConfig {
    YawRateConfig();

    float maximum_yaw_rate_rads;
    float proportional_gain;
    float integral_gain;
    float derivative_gain;
    float derivative_cutoff_hz;
    float integral_limit;
    bool positive_steering_turns_left;
    uint32_t maximum_gyro_age_us = 50'000;
    bool valid() const;
};

struct YawRateTerms {
    float requested_rads = 0, measured_rads = 0;
    float proportional = 0, integral = 0, derivative = 0, output = 0;
    uint32_t gyro_age_us = 0;
    bool active = false;
};

class YawRateController {
public:
    explicit YawRateController(const YawRateConfig& config) : config_(config) {}

    float update(float requested_yaw_rate_rads, float measured_yaw_rate_rads,
                 int64_t now_us, float minimum_steering, float maximum_steering,
                 bool integrate);
    void reset();
    void configure(const YawRateConfig& config) { config_ = config; reset(); }
    const YawRateConfig& config() const { return config_; }
    const YawRateTerms& terms() const { return terms_; }

private:
    YawRateConfig config_;
    YawRateTerms terms_{};
    float integral_correction_ = 0.0f;
    float previous_measurement_ = 0.0f;
    float filtered_derivative_ = 0.0f;
    bool previous_measurement_valid_ = false;
    int64_t last_update_us_ = -1;
};

} // namespace vehicle
