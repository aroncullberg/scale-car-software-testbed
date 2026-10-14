# Motor Component Refactor Plan

## Context

Replace `aroncullberg__EscDriver` managed component with a new `components/motor/` component. Clean API: `motor.init()`, `motor.throttle(0.5f)`, `motor.cmd(motor::Command::BEEP1)`. Arming is automatic and non-blocking, commands auto-handle repetitions and stop-before-switch, throttle gracefully returns false during arming.

Two layers:
- **Layer 1 (`DshotDriver`)**: Stateless DShot protocol I/O over RMT. No tasks, no state machine. Uses the existing `dshot_rmt_encoder.c/.h` from the managed component (copied into the new component).
- **Layer 2 (`Motor`)**: Owns a DshotDriver + background FreeRTOS task at 500Hz. State machine handles arming, throttle, commands.

---

## New Files

### `components/motor/include/dshot_types.h`
Enums, constants, config structs:
- `motor::Command` enum: MOTOR_STOP=0, BEEP1-5, SPIN_DIRECTION_1=7, SPIN_DIRECTION_2=8, MODE_3D_OFF=9, MODE_3D_ON=10, SAVE_SETTINGS=12, SPIN_DIRECTION_NORMAL=20, SPIN_DIRECTION_REVERSED=21
- `motor::State` enum: UNINITIALIZED, ARMING, IDLE, RUNNING, COMMANDING
- `motor::MotorConfig` struct: gpio, poles(=14), bidirectional(=true)
- Constants: THROTTLE_MIN=48, THROTTLE_MAX=2047, ARM_DELAY_MS=3200

### `components/motor/include/dshot_driver.h`
Layer 1 — raw DShot protocol:
- `DshotDriver(gpio_num_t gpio, bool bidirectional)`
- `esp_err_t init()` — creates RMT TX/RX channels and encoder
- `void send_throttle(uint16_t value)` — clamps [48,2047], telemetry bit CLEAR
- `void send_command(uint16_t cmd)` — guard <48, telemetry bit SET
- `void send_idle()` — sends value 0 with telemetry bit CLEAR (for arming)
- `uint32_t erpm() const` — last decoded eRPM from bidirectional telemetry
- `static float erpm_to_rpm_ratio(uint8_t poles)`
- Private: ISR callbacks, GCR decode, mode_tx/mode_rx/enable_rx/disable_rx
- Uses `dshot_rmt_encoder.c/.h` (copied from managed component) for frame encoding via `rmt_bytes_encoder`

### `components/motor/dshot_driver.cpp`
Layer 1 implementation. Follows same architecture as existing `DShotRMT.cpp`:
- **`init()`**: Compute baudrate (600kHz for DSHOT600). Create encoder via `rmt_new_dshot_esc_encoder()`. If bidirectional: create RX channel (io_loop_back=true), register rx_done_isr, compute telemetry timing. Create TX channel (invert_out=bidirectional, io_loop_back=true, trans_queue_depth=4). If bidirectional: register tx_done_isr. Enable TX channel.
- **`send_throttle()`**: Set throttle struct fields (value, telemetry_req=false), call `rmt_transmit()`.
- **`send_command()`**: Set throttle struct (cmd value, telemetry_req=true), call `rmt_transmit()`, restore telemetry_req=false.
- **`send_idle()`**: Set throttle struct (value=0, telemetry_req=false), call `rmt_transmit()`.
- **ISR flow**: Same as existing — tx_done_isr → mode_rx() → gpio_ll_od_enable → rmt_enable(RX) → rmt_receive(). rx_done_isr → extract GCR → store atomically.
- **GCR decode**: Same three static functions from existing driver (standard DShot bidirectional protocol).

### `components/motor/dshot_rmt_encoder.c` and `dshot_rmt_encoder.h`
Copied from `managed_components/aroncullberg__EscDriver/`. These provide the `rmt_new_dshot_esc_encoder()` function and `dshot_rmt_throttle_t` / `dshot_rmt_encoder_config_t` types. Already working and tested.

### `components/motor/include/motor.h`
Layer 2 — user-facing motor abstraction:
```
Motor(const MotorConfig&)
esp_err_t init()           // starts task + arming, returns immediately
void disarm()

bool throttle(float value) // [0.0, 1.0], false if not armed
bool brake(float strength) // stub — reserved for future dithering, does nothing
bool cmd(Command)          // queues command, auto-handles stop→cmd→idle

int32_t rpm() const
uint32_t erpm() const
State state() const
bool is_armed() const
```

### `components/motor/motor.cpp`
Layer 2 implementation:

**`init()`**: Init driver, set state→ARMING, record arming_start_tick_, spawn task (2048 stack, priority 5, core 1).

**`task_loop()`** (500Hz):
```
check cmd_pending → enter COMMANDING
switch(state):
  ARMING:     tick_arming()
  IDLE:       tick_idle()
  RUNNING:    tick_running()
  COMMANDING: tick_commanding()
vTaskDelayUntil(2ms)
```

**`tick_arming()`**: `driver_.send_idle()`. After 3200ms → state=IDLE.

**`tick_idle()`**: `driver_.send_throttle(48)`. If desired_throttle_ > 48 → state=RUNNING.

**`tick_running()`**: `driver_.send_throttle(desired_throttle_)`. If desired_throttle_ ≤ 48 → state=IDLE.

**`tick_commanding()`**: Three phases handled by a counter:
1. **Pre-idle** (first ~25 frames / 50ms): send `driver_.send_idle()` — ensures motor is stopped before direction-change commands. Harmless for non-direction commands.
2. **Command** (next 10 frames): send `driver_.send_command(active_cmd_)` with telemetry bit set.
3. **Done**: state → IDLE, desired_throttle_ reset to 48.

This means `cmd()` always safely handles direction switching without the caller knowing about the stop requirement. Beep commands get a brief idle first (harmless). Direction commands get the mandatory stop.

**`throttle(float)`**: Guard state ∈ {IDLE, RUNNING}. Clamp [0,1], map to [48,2047], store atomically.

**`brake(float)`**: Stub — does nothing. Placeholder for future dithering.

**`cmd(Command)`**: Guard state not UNINITIALIZED. Store pending_cmd_, set cmd_pending_=true.

**`disarm()`**: state=DISARMED, desired_throttle_=48.

**`rpm()`**: `driver_.erpm() * erpm_to_rpm_ratio(poles)`. Return 0 on invalid.

### `components/motor/CMakeLists.txt`
```cmake
idf_component_register(
    SRCS "dshot_driver.cpp" "motor.cpp" "dshot_rmt_encoder.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
    PRIV_REQUIRES esp_timer
)
```

---

## Modified Files

### `components/vehicle/include/vehicle_controller.h`
- `#include "DShotRMT.h"` → `#include "motor.h"`
- Config: `DShotRMT&` → `motor::Motor&` for all 4 motors
- Remove `int motor_poles{14}` from Config
- Remove `std::array<uint32_t, 4> motor_erpm_`

### `components/vehicle/vehicle_controller.cpp`
- `control_motors()`: Work in float [0,1]. Call `motor.throttle(per_motor[i])`. Software differential in float space (boost capped at 1.0). Read `motor.rpm()` for telemetry. Brake override: `motor.cmd(motor::Command::MOTOR_STOP)` — actually no, brake override should just send DShot 48... actually, looking at current code it calls `motors[i]->sendCmd(0)` which is command MOTOR_STOP. But that's a one-shot per frame in the current loop. With the new API the vehicle controller runs at 500Hz and each tick it decides throttle or brake. Since brake() is a stub, for now brake override will call `motor.throttle(0)` to idle the motor. Full active-braking can be wired up when brake() is implemented.
- `handle_rc_loss()`: Call `motor.throttle(0)` on all 4 motors.
- Remove all `DShotRMT::getErpmToRpmRatio()` and raw DShot value math.

### `components/vehicle/CMakeLists.txt`
- Replace `aroncullberg__EscDriver` with `motor`

### `components/vehicle/idf_component.yml`
- Remove `aroncullberg/EscDriver` git dependency

### `main/main.cpp`
- `#include "DShotRMT.h"` → `#include "motor.h"`
- Replace 4x `DShotRMT` construction with `motor::Motor` construction using `MotorConfig{.gpio = GPIO_NUM_XX}`
- Replace `begin_UNSAFE()` + `vTaskDelay(3000)` with `motor.init()` (non-blocking)
- Update `vehicle::Controller::Config` (no motor_poles)

### `main/CMakeLists.txt`
- Remove entire DSHOT_BENCH_TEST conditional block
- Keep only the normal app build

### Delete
- `main/main_dshot_test.cpp`

---

## State Machine

```
UNINITIALIZED ──[init()]──→ ARMING ──[3.2s]──→ IDLE ←──→ RUNNING
                                                  ↑           ↑
                                                  └─── COMMANDING (any armed state)
Any state ──[disarm()]──→ DISARMED
```

- IDLE ↔ RUNNING: transitions based on desired_throttle_ being above/at THROTTLE_MIN
- COMMANDING: entered when cmd_pending_ is set. Pre-idles, sends command, returns to IDLE.
- throttle() returns false when state is ARMING, COMMANDING, UNINITIALIZED, or DISARMED.

---

## Implementation Order

1. Create `components/motor/` — copy encoder files, write all new files
2. Modify vehicle component (CMakeLists, idf_component.yml, header, cpp)
3. Modify main (main.cpp, CMakeLists.txt)
4. Delete main_dshot_test.cpp
5. Build and test

---

## Verification

1. `idf.py build` — clean build, no warnings
2. Flash → serial shows 4x "armed" after ~3.2s
3. RC throttle → motors spin, differential works
4. RPM telemetry shows on CRSF link
5. Remove RC link → motors idle
6. Per-motor enable channels (ch13-16) still work
