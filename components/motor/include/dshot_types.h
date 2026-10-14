#pragma once

#include <cstdint>
#include "driver/gpio.h"

namespace motor {

// Only the commands that survive AM32 rc_car_reverse. Direction (7/8/20/21) and
// 3D mode (9/10) are gone: car mode forces bi_direction on, and which half of
// the split range drives a given wheel is chosen host-side by Mounting.
enum class Command : uint16_t {
    MOTOR_STOP = 0,
    BEEP1 = 1,
    BEEP2 = 2,
    BEEP3 = 3,
    BEEP4 = 4,
    BEEP5 = 5,
    SAVE_SETTINGS = 12,
};

// rc_car_reverse splits the 11-bit DShot range into two 1000-step halves. The
// base value of each half is that half's zero-throttle point: AM32 computes
// ((value - base) * 2 + 47) - reversing_dead_band and drives only at >= 47, so
// base itself yields 46 and produces no torque.
//
// Commanding the half that matches the ESC's internal direction drives it.
// Commanding the opposing half brakes it, proportional to the distance past
// that half's base, and the ESC never reverses out of a brake on its own. The
// direction only flips after a frame below 48 arrives while braking, which is
// what Motor's sync sequence uses.
static constexpr uint16_t HALF_LOW_BASE = 48;
static constexpr uint16_t HALF_HIGH_BASE = 1048;
static constexpr uint16_t HALF_SPAN = 999;
static constexpr uint16_t DSHOT_NEUTRAL = 0;

static constexpr uint16_t THROTTLE_MIN = 48;
static constexpr uint16_t THROTTLE_MAX = 2047;
// AM32 arms after roughly a second of zero-throttle frames, but it only starts
// counting once it has locked onto the signal, and that lock takes a variable
// few seconds. Telemetry coming back is the only evidence we have of it, so the
// hold is timed from there. The timeout is the fallback for a wheel whose
// telemetry never arrives.
static constexpr uint32_t ARM_HOLD_MS = 1500;
static constexpr uint32_t ARM_DETECT_TIMEOUT_MS = 15000;

// Which half of the split range is this wheel's forward. Static per wheel,
// decided by how the motor is physically mounted.
enum class Mounting : uint8_t {
    NORMAL = 0,
    MIRRORED = 1,
};

// Requested travel direction. Changing it costs a sync sequence and is only
// safe at standstill, so the vehicle controller owns when it happens.
enum class Gear : uint8_t {
    FORWARD = 0,
    REVERSE = 1,
};

// The three states bidirectional DShot telemetry can be in. Collapsing Stopped
// and NoData to a single zero is what made the old rpm() unusable as a
// standstill signal.
enum class RpmStatus : uint8_t {
    NoData = 0,
    Stopped = 1,
    Valid = 2,
};

struct RpmSample {
    int32_t rpm;
    RpmStatus status;
};

struct ErpmSample {
    uint32_t erpm;
    RpmStatus status;
};

enum class State : uint8_t {
    UNINITIALIZED,
    DISARMED,
    ARMING,
    ESTABLISHING,
    READY,
    CHANGING_GEAR,
    COMMANDING,
};

struct MotorConfig {
    gpio_num_t gpio;
    uint8_t poles{14};
    Mounting mounting{Mounting::NORMAL};

    // Inverted idle-high DShot with inverted CRC, the protocol variant a
    // bidirectional-capable ESC expects. Independent of whether we read back.
    bool inverted{true};

    // Half-duplex eRPM return: RX channel, tx-done handoff, open-drain during
    // the receive window. Off leaves the line driven push-pull the whole time
    // and makes rpm() permanently NoData, which disables the gear change.
    bool telemetry{true};
};

} // namespace motor
