#include "vehicle/yaw_rate_controller.hpp"

#include <algorithm>
#include <cmath>
#include "sdkconfig.h"

namespace vehicle {

YawRateConfig::YawRateConfig()
    : maximum_yaw_rate_rads(CONFIG_CAR_YAW_RATE_MAX_DEG_S * 0.017453292519943295f),
      proportional_gain(CONFIG_CAR_YAW_RATE_PROPORTIONAL_X1000 / 1000.0f),
      integral_gain(CONFIG_CAR_YAW_RATE_INTEGRAL_X1000 / 1000.0f),
      derivative_gain(CONFIG_CAR_YAW_RATE_DERIVATIVE_X1000 / 1000.0f),
      derivative_cutoff_hz(CONFIG_CAR_YAW_RATE_DERIVATIVE_CUTOFF_HZ),
      integral_limit(CONFIG_CAR_YAW_RATE_INTEGRAL_LIMIT_PERMILLE / 1000.0f),
#ifdef CONFIG_CAR_POSITIVE_STEERING_TURNS_LEFT
      positive_steering_turns_left(true) {}
#else
      positive_steering_turns_left(false) {}
#endif

bool YawRateConfig::valid() const {
    return std::isfinite(maximum_yaw_rate_rads) && maximum_yaw_rate_rads > 0 &&
        maximum_yaw_rate_rads <= 12.566371f &&
        std::isfinite(proportional_gain) && proportional_gain >= 0 && proportional_gain <= 10 &&
        std::isfinite(integral_gain) && integral_gain >= 0 && integral_gain <= 10 &&
        std::isfinite(derivative_gain) && derivative_gain >= 0 && derivative_gain <= 10 &&
        std::isfinite(derivative_cutoff_hz) && derivative_cutoff_hz >= 1 && derivative_cutoff_hz <= 30 &&
        std::isfinite(integral_limit) && integral_limit >= 0 && integral_limit <= 1;
}

void YawRateController::reset() {
    integral_correction_ = 0.0f;
    last_update_us_ = -1;
    previous_measurement_valid_ = false;
    filtered_derivative_ = 0;
    terms_ = {};
}

float YawRateController::update(float requested_yaw_rate_rads, float measured_yaw_rate_rads,
                                int64_t now_us, float minimum_steering,
                                float maximum_steering, bool integrate) {
    if (!std::isfinite(requested_yaw_rate_rads) || !std::isfinite(measured_yaw_rate_rads) ||
        !std::isfinite(minimum_steering) || !std::isfinite(maximum_steering) ||
        minimum_steering > 0.0f || maximum_steering < 0.0f || now_us < 0) {
        reset();
        return 0.0f;
    }
    float elapsed_seconds = 0.0f;
    if (last_update_us_ >= 0) {
        const auto elapsed_us = now_us - last_update_us_;
        if (elapsed_us < 0 || elapsed_us > 100'000) reset();
        else elapsed_seconds = elapsed_us / 1'000'000.0f;
    }
    last_update_us_ = now_us;

    const float yaw_rate_error_rads = requested_yaw_rate_rads - measured_yaw_rate_rads;
    const float steering_error_rads = yaw_rate_error_rads *
        (config_.positive_steering_turns_left ? 1.0f : -1.0f);
    const float proportional_correction = config_.proportional_gain * steering_error_rads;
    if (config_.derivative_gain > 0 && previous_measurement_valid_ && elapsed_seconds > 0) {
        const float derivative = -(measured_yaw_rate_rads - previous_measurement_) / elapsed_seconds *
            (config_.positive_steering_turns_left ? 1.0f : -1.0f);
        const float time_constant = 1.0f / (6.283185307f * config_.derivative_cutoff_hz);
        const float alpha = elapsed_seconds / (time_constant + elapsed_seconds);
        filtered_derivative_ += alpha * (derivative - filtered_derivative_);
    }
    previous_measurement_ = measured_yaw_rate_rads;
    previous_measurement_valid_ = true;
    const float derivative_correction = config_.derivative_gain * filtered_derivative_;
    const float minimum_integral = std::max(-config_.integral_limit, minimum_steering);
    const float maximum_integral = std::min(config_.integral_limit, maximum_steering);
    integral_correction_ = std::clamp(integral_correction_, minimum_integral, maximum_integral);
    if (integrate && elapsed_seconds > 0.0f) {
        const float next_integral = std::clamp(integral_correction_ +
            config_.integral_gain * steering_error_rads * elapsed_seconds,
            minimum_integral, maximum_integral);
        const float next_output = proportional_correction + next_integral + derivative_correction;
        const bool pushing_past_limit =
            (next_output > maximum_steering && steering_error_rads > 0.0f) ||
            (next_output < minimum_steering && steering_error_rads < 0.0f);
        if (!pushing_past_limit) integral_correction_ = next_integral;
    }
    const float output = std::clamp(proportional_correction + integral_correction_ + derivative_correction,
                                    minimum_steering, maximum_steering);
    terms_ = {requested_yaw_rate_rads, measured_yaw_rate_rads, proportional_correction,
              integral_correction_, derivative_correction, output, 0, true};
    return output;
}

} // namespace vehicle
