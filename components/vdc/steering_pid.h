//
// Created by Aron Cullberg on 2025-03-09.
//

#ifndef STEERING_PID_H
#define STEERING_PID_H


#pragma once

#include <functional>
#include "freertos/FreeRTOS.h"
// #include "esp_err.h"
#include "sensor_types.h"
#include "quaternion.h"

class SteeringPID {
public:
    enum class ControllerState {
        ACTIVE,       // Normal operation
        SUSPENDED,    // Temporarily disabled (throttle zero)
        PICKED_UP,    // Car physically manipulated
        DISABLED      // Manually disabled via switch
    };

    struct Config {
        float kP{2.0f};                  // Proportional gain
        float kI{0.05f};                 // Integral gain
        float kD{0.1f};                  // Derivative gain
        float feedForwardGain{0.5f};     // Feedforward gain for gyro input

        float maxIntegral{1.0f};         // Anti-windup limit
        float maxSteeringAngle{1.57f};   // Maximum target steering angle (90 degrees in radians)

        float headingChangeRate{4.5f};   // Rate of change for target heading

        float accelThresholdPickup{1.5f};// Acceleration threshold to detect pickup (G)
        uint32_t throttleZeroTimeoutMs{800}; // Time with zero throttle to suspend
        uint32_t resumeStabilizeTimeMs{500}; // Stabilization time when resuming

        bool useGyroFeedforward{true};   // Enable gyro feedforward
        float gyroInfluence{0.65f};      // Influence of gyro on final output (0.0-1.0)
    };

    explicit SteeringPID(const Config& config);

    ~SteeringPID();

    /**
     * @brief Update PID controller with latest sensor data
     *
     * @param sbus_data SBUS data with steering input
     * @param imu_data IMU data with orientation and gyro data
     * @param deltaTime Time since last update in seconds
     * @return float Steering output (-1.0 to 1.0)
     */
    float update(const sensor::SbusData& sbus_data,
                 const sensor::ImuData& imu_data,
                 float deltaTime);

    void reset();

    void setState(ControllerState state);

    ControllerState getState() const { return state_; }

    void loadConfig();

    Quaternion getReferenceOrientation() const { return referenceOrientation_; }

    // void setConfigCallback(std::function<void()> callback) { configCallback_ = callback; }
    void setConfigCallback(const std::function<void()> &callback) { configCallback_ = callback; }

    void updateConfig(const Config& config);

private:
    static constexpr const char* TAG = "SteeringPID";

    Config config_;
    ControllerState state_{ControllerState::ACTIVE};

    float targetHeading_{0.0f};
    float integral_{0.0f};
    float previousError_{0.0f};
    float lastGyroRate_{0.0f};

    Quaternion referenceOrientation_{};
    uint32_t lastThrottleActiveTime_{0};
    uint32_t stateTransitionTime_{0};

    std::function<void()> configCallback_{};

    float calculateHeadingError(const Quaternion& currentOrientation) const;
    void updateTargetHeading(float steeringInput, float deltaTime);
    void updateReferenceOrientation(const sensor::SbusData& sbus_data,
                                    const sensor::ImuData& imu_data);
    bool detectPickup(const sensor::ImuData& imu_data) const;
    void checkStateTransitions(const sensor::SbusData& sbus_data,
                               const sensor::ImuData& imu_data);

    float computePTerm(float error) const;
    float computeITerm(float error, float deltaTime);
    float computeDTerm(float error, float deltaTime, float gyroRate) const;
    float computeFeedForward(float gyroRate) const;

    static Quaternion getCurrentOrientation(const sensor::ImuData& imu_data);

    // Get Z-axis gyro rate in radians/sec
    static float getGyroZRate(const sensor::ImuData& imu_data);
};


#endif //STEERING_PID_H
