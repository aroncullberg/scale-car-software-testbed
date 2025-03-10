//
// Created by Aron Cullberg on 2025-03-09.
//

#include "steering_pid.h"

#include <cmath>
#include "esp_log.h"
#include "config_manager.h"


SteeringPID::SteeringPID(const Config& config) : config_(config) {
    loadConfig();
}


SteeringPID::~SteeringPID() = default;

void SteeringPID::loadConfig() {
    config_.kP = ConfigManager::instance().getFloat("pid/kp", config_.kP);
    config_.kI = ConfigManager::instance().getFloat("pid/ki", config_.kI);
    config_.kD = ConfigManager::instance().getFloat("pid/kd", config_.kD);
    config_.feedForwardGain = ConfigManager::instance().getFloat("pid/ff", config_.feedForwardGain);
    config_.gyroInfluence = ConfigManager::instance().getFloat("pid/gyro_influence", config_.gyroInfluence);
    config_.headingChangeRate = ConfigManager::instance().getFloat("pid/heading_rate", config_.headingChangeRate);

    ESP_LOGI(TAG, "Loaded PID config: P=%.2f, I=%.2f, D=%.2f, FF=%.2f, Influence=%.2f, Rate=%.2f",
             config_.kP, config_.kI, config_.kD, config_.feedForwardGain,
             config_.gyroInfluence, config_.headingChangeRate);
}

void SteeringPID::updateConfig(const Config& config) {
    config_ = config;
    if (configCallback_) {
        configCallback_();
    }
}

float SteeringPID::update(const sensor::SbusData& sbus_data,
                          const sensor::ImuData& imu_data,
                          float deltaTime) {
    if (state_ == ControllerState::DISABLED) {
        return 0.0f;
    }

    checkStateTransitions(sbus_data, imu_data);

    // Skip processing if in non-active states
    if (state_ != ControllerState::ACTIVE) {
        // Just pass through direct control during non-active states
        float steeringInput = static_cast<float>(sbus_data.channels[static_cast<size_t>(sensor::SbusChannel::STEERING)] - 1500) / 500.0f;
        return steeringInput;
    }

    // Update reference orientation if needed
    updateReferenceOrientation(sbus_data, imu_data);

    // Get current orientation and steering input
    Quaternion currentOrientation = getCurrentOrientation(imu_data);
    float steeringInput = static_cast<float>(sbus_data.channels[static_cast<size_t>(sensor::SbusChannel::STEERING)] - 1500) / 500.0f;

    // Update target heading based on steering input
    updateTargetHeading(steeringInput, deltaTime);

    // Calculate heading error
    float headingError = calculateHeadingError(currentOrientation);

    // Get gyro Z rate for D-term and feed-forward
    float gyroRate = getGyroZRate(imu_data);

    // Compute PID terms
    float pTerm = computePTerm(headingError);
    float iTerm = computeITerm(headingError, deltaTime);
    float dTerm = computeDTerm(headingError, deltaTime, gyroRate);
    float feedForward = computeFeedForward(gyroRate);

    // Combine terms for final correction
    float correction = pTerm + iTerm + dTerm + feedForward;

    // Mix direct control with PID correction based on gyro influence
    float directFactor = 1.0f - config_.gyroInfluence;
    float gyroFactor = config_.gyroInfluence;
    float finalSteering = steeringInput * directFactor - correction * gyroFactor;

    // Debug logging (throttled to avoid flooding)
    static uint32_t lastLogTime = 0;
    uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (currentTime - lastLogTime > 500) {  // Log every 500ms
        ESP_LOGD(TAG, "PID: err=%.2f, P=%.2f, I=%.2f, D=%.2f, FF=%.2f, out=%.2f",
                 headingError, pTerm, iTerm, dTerm, feedForward, finalSteering);
        lastLogTime = currentTime;
    }

    // Clamp to valid range
    if (finalSteering > 1.0f) finalSteering = 1.0f;
    if (finalSteering < -1.0f) finalSteering = -1.0f;

    // Store values for next iteration
    previousError_ = headingError;
    lastGyroRate_ = gyroRate;

    return finalSteering;
}

void SteeringPID::reset() {
    // Reset controller state
    targetHeading_ = 0.0f;
    integral_ = 0.0f;
    previousError_ = 0.0f;
    lastGyroRate_ = 0.0f;
    state_ = ControllerState::ACTIVE;

    ESP_LOGI(TAG, "PID controller reset");
}

void SteeringPID::setState(ControllerState state) {
    if (state_ == state) {
        return;
    }

    ESP_LOGI(TAG, "PID state change: %d -> %d", static_cast<int>(state_), static_cast<int>(state));

    // Record time of state transition
    stateTransitionTime_ = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Reset integral on state changes
    if (state == ControllerState::ACTIVE &&
        (state_ == ControllerState::SUSPENDED || state_ == ControllerState::PICKED_UP)) {
        integral_ = 0.0f;
    }

    state_ = state;
}

Quaternion SteeringPID::getCurrentOrientation(const sensor::ImuData& imu_data) {
    return Quaternion::fromGameVector(imu_data.quat6_x, imu_data.quat6_y, imu_data.quat6_z);
}

float SteeringPID::getGyroZRate(const sensor::ImuData& imu_data) {
    // Convert gyro Z data to radians/second
    // Note: This conversion factor depends on your IMU settings
    // For DPS_500 setting: 1 unit = 1/64 dps, convert to rad/s
    constexpr float DPS_TO_RADS = M_PI / 180.0f;
    constexpr float GYRO_SCALE = 1.0f / 64.0f * DPS_TO_RADS;

    return static_cast<float>(imu_data.gyro_z) * GYRO_SCALE;
}

float SteeringPID::calculateHeadingError(const Quaternion& currentOrientation) const {
    // Calculate heading difference between reference and current orientation
    float baseHeading = Quaternion::headingDifference(referenceOrientation_, currentOrientation);

    // Calculate error between target and current heading
    float error = targetHeading_ - baseHeading;

    // Normalize to -PI to +PI
    while (error > M_PI) error -= 2.0f * M_PI;
    while (error < -M_PI) error += 2.0f * M_PI;

    // Limit error to maxSteeringAngle (prevents extreme corrections)
    if (error > config_.maxSteeringAngle) error = config_.maxSteeringAngle;
    if (error < -config_.maxSteeringAngle) error = -config_.maxSteeringAngle;

    return error;
}

void SteeringPID::updateTargetHeading(float steeringInput, float deltaTime) {
    // Apply small deadzone for stability
    if (std::fabs(steeringInput) > 0.05f) {
        // Change rate proportional to stick deflection
        float headingChangeRate = -steeringInput * config_.headingChangeRate;
        targetHeading_ += headingChangeRate * deltaTime;

        // Normalize target heading
        while (targetHeading_ > M_PI) targetHeading_ -= 2.0f * M_PI;
        while (targetHeading_ < -M_PI) targetHeading_ += 2.0f * M_PI;
    }
}

void SteeringPID::updateReferenceOrientation(const sensor::SbusData& sbus_data,
                                            const sensor::ImuData& imu_data) {
    // Check if throttle is active
    constexpr auto THROTTLE_CHANNEL = static_cast<size_t>(sensor::SbusChannel::THROTTLE);
    const bool throttleActive = sbus_data.channels[THROTTLE_CHANNEL] > 1050;  // Small threshold
    const uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (throttleActive) {
        lastThrottleActiveTime_ = currentTime;
    } else {
        // If throttle has been inactive for the timeout period, reset reference
        if ((currentTime - lastThrottleActiveTime_) > config_.throttleZeroTimeoutMs) {
            // Only reset if we haven't recently done so
            if (currentTime - stateTransitionTime_ > config_.resumeStabilizeTimeMs) {
                referenceOrientation_ = getCurrentOrientation(imu_data);
                targetHeading_ = 0.0f;
                integral_ = 0.0f;

                ESP_LOGD(TAG, "Reference orientation reset due to throttle timeout");
            }
        }
    }
}

bool SteeringPID::detectPickup(const sensor::ImuData& imu_data) const {
    // NOTE: For GPM_4 setting, 1 unit = 1/8192 g
    constexpr float ACCEL_SCALE = 1.0f / 8192.0f;

    float accelZ = static_cast<float>(imu_data.accel_z) * ACCEL_SCALE;

    float accelMagnitude = std::abs(accelZ);

    bool isPickedUp = std::fabs(accelMagnitude - 1.0f) > config_.accelThresholdPickup;

    return isPickedUp;
}

void SteeringPID::checkStateTransitions(const sensor::SbusData& sbus_data,
                                       const sensor::ImuData& imu_data) {
    const uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

    bool isPickedUp = detectPickup(imu_data);

    constexpr auto THROTTLE_CHANNEL = static_cast<size_t>(sensor::SbusChannel::THROTTLE);
    const bool throttleActive = sbus_data.channels[THROTTLE_CHANNEL] > 1010;

    switch (state_) {
        case ControllerState::ACTIVE:
            if (isPickedUp) {
                setState(ControllerState::PICKED_UP);
                ESP_LOGI(TAG, "Picked up detected - transitioning to PICKED_UP state");
            } else if (!throttleActive &&
                      (currentTime - lastThrottleActiveTime_ > config_.throttleZeroTimeoutMs)) {
                setState(ControllerState::SUSPENDED);
                ESP_LOGI(TAG, "Throttle inactive - transitioning to SUSPENDED state");
            }
            break;

        case ControllerState::SUSPENDED:
            if (isPickedUp) {
                setState(ControllerState::PICKED_UP);
                ESP_LOGI(TAG, "Picked up detected - transitioning to PICKED_UP state");
            } else if (throttleActive) {
                // If enough time has passed since suspension, go back to active
                if (currentTime - stateTransitionTime_ > config_.resumeStabilizeTimeMs) {
                    // Reset reference before going active
                    referenceOrientation_ = getCurrentOrientation(imu_data);
                    targetHeading_ = 0.0f;
                    setState(ControllerState::ACTIVE);
                    ESP_LOGI(TAG, "Throttle active - transitioning to ACTIVE state");
                }
            }
            break;

        case ControllerState::PICKED_UP:
            if (!isPickedUp) {
                // If not picked up for stabilization period, transition
                if (currentTime - stateTransitionTime_ > config_.resumeStabilizeTimeMs) {
                    if (throttleActive) {
                        // Reset reference before going active
                        referenceOrientation_ = getCurrentOrientation(imu_data);
                        targetHeading_ = 0.0f;
                        setState(ControllerState::ACTIVE);
                        ESP_LOGI(TAG, "Transitioning to ACTIVE state from PICKED_UP");
                    } else {
                        setState(ControllerState::SUSPENDED);
                        ESP_LOGI(TAG, "Transitioning to SUSPENDED state from PICKED_UP");
                    }
                }
            }
            break;

        case ControllerState::DISABLED:
            // Only transition out of DISABLED through explicit setState call
            break;
    }
}

float SteeringPID::computePTerm(float error) const {
    return error * config_.kP;
}

float SteeringPID::computeITerm(float error, float deltaTime) {
    // Update integral term with anti-windup
    integral_ += error * deltaTime;

    // Apply anti-windup (clamp integral)
    if (integral_ > config_.maxIntegral) integral_ = config_.maxIntegral;
    if (integral_ < -config_.maxIntegral) integral_ = -config_.maxIntegral;

    return integral_ * config_.kI;
}

float SteeringPID::computeDTerm(float error, float deltaTime, float gyroRate) const {
    // There are two options for the D term:
    // 1. Traditional derivative of error
    // 2. Direct use of gyro rate (cleaner signal)

    float dTerm;

    if (config_.useGyroFeedforward) {
        // Use gyro directly for D-term (negative because we want to dampen rotation)
        dTerm = -gyroRate * config_.kD;
    } else {
        // Traditional D-term: derivative of error
        float derivative = (error - previousError_) / deltaTime;
        dTerm = derivative * config_.kD;
    }

    return dTerm;
}

float SteeringPID::computeFeedForward(float gyroRate) const {
    // Feed-forward term directly uses gyro rate to predict and counteract rotation
    // Only apply if enabled
    if (config_.useGyroFeedforward) {
        return -gyroRate * config_.feedForwardGain;
    }
    return 0.0f;
}




