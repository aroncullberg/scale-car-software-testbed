# motor

Four independent DShot600 links, one per ESC, driven at 500 Hz from a task per
motor. Bidirectional DShot is mandatory here, not optional: the eRPM return is
the only standstill signal the gear change has.

Target firmware is AM32 with `rc_car_reverse = 1`. The mapping below is derived
from `Src/main.c` and `Src/dshot.c` on v2.21.

## Value mapping

`rc_car_reverse` splits the 11-bit range into two 1000-step halves. AM32 turns a
value into `((value - base) * 2 + 47) - reversing_dead_band` and only drives at
47 or above, so the base of each half is its zero point and produces no torque.

Each wheel owns a `Mounting` (which half is its physical forward) and a `Gear`
(requested travel direction). Together they pick the drive half; the other half
is the brake half.

| `command()` | DShot sent | ESC does |
|---|---|---|
| `+1 .. +999` | drive base + n | drives, duty `map(input, 47, 2047, minimum_duty_cycle, 2000)` |
| `0` | drive base | freewheels, both FETs off with a phase floating |
| `-1 .. -977` | brake base + n | `proportionalBrake()`, duty `n * 2 + 46` out of 2000 |
| `-978 .. -999` | brake base + n | `fullBrake()`, hard short |

Brake and drive are the same actuator and AM32 cannot blend them, which is why
one signed value carries both.

## The direction latch

Commanding the half that opposes the ESC's internal direction brakes the wheel
and holds there indefinitely. The ESC never reverses out of a brake by itself.
It flips only after a frame below 48 arrives *while braking*, which sets AM32's
`return_to_center`, and the next opposing-half command then consumes it.

That makes DShot 0 the one value that can change direction behind your back, so
it is emitted from exactly two places, both inside `Motor`: the arming sequence
and `tick_syncing()`. The drive path never sends it, which is why coast is the
drive half's base rather than 0.

`tick_syncing()` is three phases against that latch: command the target half,
drop below 48, command the target half again. A wheel already facing the target
way passes through with no effect, so the same sequence serves both the
post-arm establish and a gear change. Each phase repeats `SYNC_PHASE_FRAMES`
times because a dropped neutral frame would leave the ESC unflipped.

Gear changes are only safe at standstill and `Motor` does not check that. The
vehicle controller owns the decision and flips all four together.

## eRPM

`RpmStatus` separates three things the old API collapsed into a plain zero:
`Valid` (a decoded reading), `Stopped` (AM32's 0x0fff code), `NoData` (CRC
failure, or nothing received inside `TELEMETRY_TIMEOUT_US`). The standstill gate
needs `Stopped` to mean something different from `NoData`, since telemetry is
unreliable under proportional braking and a silent ESC must never read as a
stopped wheel.

## ESC settings

`rc_car_reverse = 1` forces `bi_direction`, clears `comp_pwm`, `use_sine_start`,
`variable_pwm`, `brake_on_zero_throttle` and `stuck_rotor_protection`, and adds
50 to the three duty floors. `brake_on_stop` is *not* forced and should be 0, or
the wheels are hard-shorted whenever stopped with no brake commanded.

Extended DShot telemetry is not enabled. Command 13 must not be sent until
`DshotDriver` demuxes the frame type, or temperature frames decode as eRPM.

`dshot_rmt_encoder.c` came from
https://github.com/MichelJansson/DShot-ESP32RMT.
