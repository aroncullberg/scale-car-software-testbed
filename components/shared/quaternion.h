#pragma once

#include "math.h"

struct Quaternion {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    
    // Convert from the ICM20948 fixed-point format
    static Quaternion fromGameVector(int32_t qx, int32_t qy, int32_t qz) {
        // ICM20948 uses Q30 format for the vector part, w is implicit
        constexpr float Q30_TO_FLOAT = 1.0f / (1 << 30);
        
        // Calculate w from x,y,z (ICM provides normalized quaternions)
        float vx = qx * Q30_TO_FLOAT;
        float vy = qy * Q30_TO_FLOAT;
        float vz = qz * Q30_TO_FLOAT;
        
        // w = sqrt(1 - (x² + y² + z²))
        float w_squared = 1.0f - (vx*vx + vy*vy + vz*vz);
        float w = (w_squared > 0.0f) ? sqrtf(w_squared) : 0.0f;
        
        return {w, vx, vy, vz};
    }
    
    // Calculate yaw difference between quaternions (for heading)
    static float headingDifference(const Quaternion& q1, const Quaternion& q2) {
        // Calculate relative quaternion
        Quaternion relative = multiply(inverse(q1), q2);
        
        // Extract yaw angle (rotation around Z axis)
        return atan2f(2.0f * (relative.w * relative.z + relative.x * relative.y),
                     1.0f - 2.0f * (relative.y * relative.y + relative.z * relative.z));
    }
    
    // Basic quaternion operations
    static Quaternion inverse(const Quaternion& q) {
        return {q.w, -q.x, -q.y, -q.z};
    }
    
    static Quaternion multiply(const Quaternion& q1, const Quaternion& q2) {
        return {
            q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
            q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
            q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
            q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        };
    }
};