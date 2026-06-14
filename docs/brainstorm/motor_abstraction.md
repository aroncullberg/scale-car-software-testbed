# Motor Abstraction – Brainstorm

## The Problem

Right now motor control is **spread across three layers** with no clear ownership boundary:

| Concern | Where it lives today |
|---|---|
| GPIO pin assignment & DShot mode | `main.cpp` – raw `DShotRMT` constructors |
| Hardware init (`begin_UNSAFE`) | `main.cpp` – caller just hopes timing is ok |
| Arming sequence | Skipped (`begin_UNSAFE`), or buried inside `DShotRMT::begin()` with a hard-coded 3.2 s delay |
| Throttle → DShot value mapping | `vehicle_controller.cpp` `control_motors()` – manual math (`48 + thr/2` reverse, `1048 + thr/2` forward) |
| 3D-mode aware forward/reverse | `vehicle_controller.cpp` – inline if/else |
| Per-motor enable/disable (ch13-16) | `vehicle_controller.cpp` – inline channel checks |
| eRPM telemetry read + RPM conversion | `vehicle_controller.cpp` – inline loop |
| Safety / RC-loss disarm | `vehicle_controller.cpp` `handle_rc_loss()` – sets `armed_ = false` but doesn't actively command motors to stop |

The consequence:

- You can't use a motor without copy-pasting the DShot value math.
- Arming is either dangerous (skipped) or blocking.
- 3D-mode details (throttle ranges 48-1047 reverse, 1048-2047 forward) are **baked into the controller**, not the motor.
- There's no watchdog — if the control task hangs, motors keep spinning at last throttle.

---

## What a `Motor` Object Should Own

A new `Motor` class wraps one `DShotRMT` instance and owns **everything below the "I want X% throttle" line**.

### Core Responsibilities

```
Motor motor(gpio, poles, dshot_mode);

motor.init();          // configures RMT
motor.arm();           // non-blocking, returns immediately, motor handles arming timing internally
motor.set_throttle(0.0 .. 1.0, direction);   // maps to correct 3D-mode DShot value
motor.stop();          // commands zero throttle (DShot idle), not the same as disarm
motor.disarm();        // puts motor in safe state

float rpm = motor.get_rpm();   // already pole-corrected
Motor::State s = motor.state();  // UNINITIALIZED | ARMING | ARMED | DISARMED | ERROR
```

### Internal Concerns the Motor Hides

1. **3D-mode DShot mapping** — forward range `1048..2047`, reverse range `48..1047`. The caller just says `set_throttle(0.75f, Motor::FORWARD)`.
2. **Arming state machine** — the ESC requires N ms of zero-throttle before it accepts commands. `Motor` tracks this internally. If you call `set_throttle()` before arming completes it either queues or returns an error.
3. **Watchdog timer** — if no `set_throttle()` call arrives within a configurable timeout (default ~50 ms), the motor automatically sends zero-throttle. This is the single most important safety feature.
4. **eRPM → RPM** — the pole-count conversion lives here, not in the controller.
5. **Telemetry** — `get_rpm()` returns the last known value, or 0 / NaN if invalid. No `65535` sentinel leaking out.

---

## Proposed Interface (sketch)

```cpp
namespace motor {

enum class Direction : uint8_t { FORWARD, REVERSE };

enum class State : uint8_t {
    UNINITIALIZED,
    ARMING,         // sending zero-throttle for arm delay
    IDLE,           // armed, accepting commands, outputting zero
    RUNNING,        // non-zero throttle being sent
    ERROR           // RMT init failed, watchdog tripped, etc.
};

struct Config {
    gpio_num_t      gpio;
    dshot_mode_t    mode         = DSHOT600_BIDIRECTIONAL;
    uint8_t         poles        = 14;
    uint32_t        arm_delay_ms = 3200;  // ESC arming time
    uint32_t        watchdog_ms  = 50;    // auto-stop if no command within this window
};

class Motor {
public:
    explicit Motor(const Config& cfg);
    ~Motor();

    // --- lifecycle ---
    esp_err_t init();                // allocate RMT resources
    esp_err_t arm();                 // start arming sequence (non-blocking)
    void      disarm();              // go to safe state immediately

    // --- commands ---
    void set_throttle(float throttle, Direction dir = Direction::FORWARD);
    //   throttle: 0.0 = idle,  1.0 = full
    //   Internally maps to correct DShot 3D range.
    //   Resets watchdog timer on every call.

    void stop();                     // equivalent to set_throttle(0)

    // --- telemetry ---
    float    get_rpm() const;        // pole-corrected RPM, 0 if unknown
    uint32_t get_erpm_raw() const;   // raw eRPM for diagnostics
    State    state()   const;

    // --- internals (would be private) ---
    // Watchdog: runs in a FreeRTOS timer or checked from the caller's loop.
    // If no set_throttle() for > watchdog_ms → send DShot 0 and transition to IDLE.
};

} // namespace motor
```

---

## How It Simplifies `main.cpp`

**Before:**
```cpp
static DShotRMT motor_fl(GPIO_NUM_41, DSHOT600_BIDIRECTIONAL);
static DShotRMT motor_fr(GPIO_NUM_39, DSHOT600_BIDIRECTIONAL);
static DShotRMT motor_rl(GPIO_NUM_38, DSHOT600_BIDIRECTIONAL);
static DShotRMT motor_rr(GPIO_NUM_40, DSHOT600_BIDIRECTIONAL);
motor_fl.begin_UNSAFE();
motor_fr.begin_UNSAFE();
motor_rl.begin_UNSAFE();
motor_rr.begin_UNSAFE();
vTaskDelay(pdMS_TO_TICKS(3000));   // hope this is enough...
```

**After:**
```cpp
static motor::Motor motor_fl({.gpio = GPIO_NUM_41});
static motor::Motor motor_fr({.gpio = GPIO_NUM_39});
static motor::Motor motor_rl({.gpio = GPIO_NUM_38});
static motor::Motor motor_rr({.gpio = GPIO_NUM_40});

for (auto* m : {&motor_fl, &motor_fr, &motor_rl, &motor_rr}) {
    m->init();
    m->arm();  // non-blocking, internally tracks arming state
}
// motors handle their own arming timing
```

---

## How It Simplifies `vehicle_controller.cpp`

**Before** (the 3D-mode math in `control_motors()`):
```cpp
int16_t dshot_value = reverse
    ? 48 + throttle_input / 2
    : 1048 + throttle_input / 2;
dshot_value = std::clamp(dshot_value, (int16_t)DSHOT_THROTTLE_MIN, (int16_t)DSHOT_THROTTLE_MAX);
// ... per-motor enable checks ...
motors[i]->sendThrottle(enabled[i] ? motor_dshot[i] : 1048);
uint32_t erpm = motors[i]->getErpm();
motor_erpm_[i] = erpm != 65535 ? erpm : 0;
```

**After:**
```cpp
float throttle = throttle_input / 2000.0f;  // normalize to 0..1
auto  dir      = reverse ? motor::Direction::REVERSE : motor::Direction::FORWARD;

for (int i = 0; i < 4; i++) {
    if (enabled[i])
        motors_[i]->set_throttle(throttle_per_motor[i], dir);
    else
        motors_[i]->stop();
}

// RPM is just:
float rpm = motors_[i]->get_rpm();
```

No DShot constants, no sentinel checks, no 3D mode awareness needed at the controller level.

---

## Open Questions / Things to Think About

### 1. Arming Strategy
- **Option A**: `arm()` is non-blocking, motor internally sends zero-throttle for `arm_delay_ms` in its own task/timer, transitions to `IDLE` when done. Pro: main doesn't block. Con: needs a background task or timer per motor (or one shared).
- **Option B**: `arm()` is cooperative — caller must call `motor.tick()` periodically during arming phase and the motor counts elapsed time. Pro: no extra tasks. Con: if caller forgets to tick, arming never completes.
- **Option C**: `arm_blocking()` exists for simple cases, `arm()` for async. Both are thin wrappers.
- **Leaning towards Option A** with a single shared FreeRTOS timer or a tiny arming task that all 4 motors share.

### 2. Watchdog Implementation
- **Software timer** (FreeRTOS `xTimerCreate`) — lightweight, one per motor, callback sends zero-throttle.
- **Checked in tick** — if using a cooperative model, check elapsed time since last command.
- Could also be implemented by the caller (vehicle_controller) but that defeats the purpose of the abstraction — **the motor itself should guarantee safety**.

### 3. 3D Mode — Can We Avoid It?
> *"best would be if we could move away from using 3D mode"*

- 3D mode is currently used for **reverse**. Without 3D mode, the ESC only spins in one direction.
- To get reverse without 3D mode you'd need either:
  - **Separate ESCs that natively support reverse** (not DShot 3D), or
  - **Physical relay/H-bridge** to swap motor leads, or
  - **Just not have reverse** (acceptable for some use-cases).
- For now, 3D mode stays BUT the `Motor` abstraction hides it. If we later move to non-3D ESCs, only the `Motor` internals change — the caller's `set_throttle(0.5, FORWARD)` call stays the same. **This is the main win of abstracting it.**

### 4. MotorGroup / MotorArray?
Should there be a `MotorGroup` that manages 4 motors together?

```cpp
motor::MotorGroup drivetrain({&motor_fl, &motor_fr, &motor_rl, &motor_rr});
drivetrain.arm_all();
drivetrain.set_all(0.5f, Direction::FORWARD);
drivetrain.set_individual({0.5f, 0.6f, 0.5f, 0.6f}, Direction::FORWARD);  // differential
```

- Pro: single point for safety (one watchdog for the whole group), simpler init.
- Con: over-engineering? Maybe just an array of `Motor*` in the controller is enough.
- **Leaning towards keeping it simple** — just `Motor` objects, the controller holds an array/struct of them.

### 5. Where Does Differential Logic Live?
- Currently in `vehicle_controller.cpp` as software diff based on RC channels.
- This should **stay in the controller**, not in the Motor class. The motor doesn't care about steering — it just gets a throttle value.
- The controller computes per-motor throttle values (with diff boost) and then calls `motor.set_throttle()` for each.

### 6. DShot Commands (0-47)
- DShot has special commands for things like setting motor direction, saving settings, beeping, LED control, etc.
- Should `Motor` expose `send_command(DshotCommand cmd)`?
- Useful for: setting 3D mode on the ESC, beep sequences for finding the vehicle, saving motor direction.
- Can be added later, not critical for v1.

---

## Suggested File Layout

```
components/
  motor/
    include/
      motor.h              // Motor class declaration
    motor.cpp              // implementation
    CMakeLists.txt
    SUMMARY.md
```

The `motor` component depends on `aroncullberg__EscDriver` (DShotRMT) and FreeRTOS.  
The `vehicle` component depends on `motor` instead of directly on `EscDriver`.

---

## Priority / Next Steps

1. **Agree on the interface** — does this sketch feel right?
2. **Decide arming strategy** — Option A (background), B (cooperative), or C (both)?
3. **Implement `Motor` class** — thin wrapper, most of the DShot math is already written, just relocating it.
4. **Refactor `vehicle_controller`** — use `Motor` instead of raw `DShotRMT`.
5. **Add watchdog** — probably the most important safety improvement.
6. **Tests** — hard on embedded, but at least static analysis of the state machine.
