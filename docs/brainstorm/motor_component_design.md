# Motor Component Design — Phase 2

## Goal

Replace raw `DShotRMT` usage with a `motor::Motor` abstraction that:
- Hides 3D-mode / direction-switching internals
- Gives full 48-2047 throttle range (normal mode, not 3D) — 2x resolution vs current 3D setup
- Handles arming, command repetition, and direction state internally
- Makes the vehicle controller only think in terms of `set_throttle(float, Direction)`

## Direction Strategy: Normal Mode + DShot Commands

Instead of running in 3D mode (which halves throttle resolution to 1000 steps per direction), run ESCs in **normal mode** and use DShot direction commands to reverse:

| Command | Value | Behavior |
|---|---|---|
| `SPIN_DIRECTION_1` | 7 | Temporary (until power cycle) |
| `SPIN_DIRECTION_2` | 8 | Temporary (until power cycle) |
| `SPIN_DIRECTION_NORMAL` | 20 | Persistent (survives power cycle) |
| `SPIN_DIRECTION_REVERSED` | 21 | Persistent (survives power cycle) |

Commands require 10+ repetitions to be accepted by the ESC. At 500Hz control loop, that's ~20ms transition time — negligible since the car must decelerate to near-zero before reversing anyway.

**Throttle ranges:**
- Normal mode: 48-2047 (2000 steps, full resolution)
- 3D mode: 48-1047 reverse / 1048-2047 forward (1000 steps each)

## Proposed Interface

```cpp
namespace motor {

enum class Direction : uint8_t { FORWARD, REVERSE };

enum class State : uint8_t {
    UNINITIALIZED,
    ARMING,
    IDLE,           // armed, zero throttle
    RUNNING,        // non-zero throttle active
    CHANGING_DIR,   // sending direction command sequence
    ERROR
};

class Motor {
public:
    Motor(gpio_num_t gpio, uint8_t poles = 14,
          dshot_mode_t mode = DSHOT600_BIDIRECTIONAL);

    esp_err_t init();
    void arm();             // blocking, 3.2s zero-throttle

    // --- throttle ---
    // throttle: 0.0 = idle, 1.0 = full
    // If direction changed, internally sends direction command x10
    // then starts sending throttle. Returns false if direction change
    // is still in progress.
    bool set_throttle(float throttle, Direction dir = Direction::FORWARD);
    void stop();            // send idle throttle (zero rotation)

    // --- commands ---
    void send_command(uint16_t cmd, uint16_t repeat = 10);

    // --- telemetry ---
    float get_rpm() const;      // pole-corrected, 0 if invalid
    uint32_t get_erpm() const;  // raw, for diagnostics
    State state() const;
    Direction direction() const;

private:
    DShotRMT dshot_;
    State state_ = State::UNINITIALIZED;
    Direction current_dir_ = Direction::FORWARD;
    uint8_t poles_;
    uint8_t dir_cmd_remaining_ = 0;  // countdown for direction change sequence
};

} // namespace motor
```

## Direction Change Flow

When `set_throttle(t, REVERSE)` is called while `current_dir_ == FORWARD`:

1. Motor sends idle throttle (stop spinning)
2. Sets `state_ = CHANGING_DIR`, `dir_cmd_remaining_ = 10`
3. Each subsequent call to `set_throttle()` during `CHANGING_DIR`:
   - Sends `SPIN_DIRECTION_2` (or `SPIN_DIRECTION_REVERSED`) instead of throttle
   - Decrements `dir_cmd_remaining_`
   - Returns `false` (throttle not applied yet)
4. When `dir_cmd_remaining_` hits 0:
   - Sets `current_dir_ = REVERSE`, `state_ = RUNNING`
   - Starts sending actual throttle
   - Returns `true`

This is cooperative — the 500Hz control loop drives the direction change without needing a separate task. 10 calls at 2ms = 20ms total transition, invisible to the driver.

**Alternative**: handle it all inside a single blocking `send_command()` call with internal delays. Simpler but blocks the control loop for 20ms. Acceptable on the bench, less ideal in production.

## Arming

Blocking `arm()` that calls `DShotRMT::begin()` (3.2s zero-throttle). Only happens at boot — no reason to make this async. If a `set_throttle()` arrives before `arm()` completes, it's ignored.

## Watchdog

Configurable but **not aggressive**. If no `set_throttle()` for 200ms, send idle and log a warning. The primary safety mechanism remains RC-loss detection in the vehicle controller. The watchdog is a last-resort backup, not the primary safety layer.

## What the Vehicle Controller Becomes

```cpp
void Controller::control_motors(...) {
    float throttle = throttle_input / 2000.0f;
    auto dir = reverse ? motor::Direction::REVERSE : motor::Direction::FORWARD;

    // differential boost per motor (stays in controller)
    float per_motor[4] = { throttle, throttle, throttle, throttle };
    if (diff_enabled) apply_differential(per_motor, ...);

    for (int i = 0; i < 4; i++) {
        if (enabled[i])
            motors_[i]->set_throttle(per_motor[i], dir);
        else
            motors_[i]->stop();
    }
}
```

No DShot constants, no 3D-mode math, no sentinel value checks.

## File Layout

```
components/
  motor/
    include/motor.h
    motor.cpp
    CMakeLists.txt
```

Depends on `aroncullberg__EscDriver`. The `vehicle` component depends on `motor` instead of `EscDriver` directly.

## Implementation Order

1. Fix EscDriver bugs (telemetry bit, off-by-one, add repetition)
2. Verify commands work on bench (beep test, direction test)
3. Build Motor component with direction-switching logic
4. Disable 3D mode on ESCs (via BLHeli or DShot command 9 = MODE_3D_OFF)
5. Refactor vehicle controller to use Motor
6. Verify full throttle range and direction switching on bench
