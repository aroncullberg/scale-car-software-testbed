# AM32 on this vehicle

Researched 2026-08-22. Firmware behaviour is read from the AM32 `main` branch
sources (`Src/main.c`, `Src/dshot.c`), because the wiki and the blogs disagree
with each other and with the code on several settings. Line numbers are from
`main` as of that date.

Current state: 4 motors on DShot over RMT, bidirectional with GCR eRPM decode
working (`components/motor/dshot_driver.cpp`), 500 Hz control loop, reverse via
DShot direction commands plus a decelerate/standstill state machine
(`Motor::tick_reversing`), brake as a 50 Hz dither between DShot 0 and a low
throttle (`BRAKE_DITHER_PERIOD = 10`, `components/motor/motor.cpp:208`).

Baseline is Bluejay on EFM8. Bluejay sources cited are
`bird-sanctuary/bluejay` at v0.21.1.

## What is checked and what is not

Read from source and quoted below, so treat as solid. Every AM32 claim with a
`Src/main.c`, `Src/dshot.c` or `Inc/eeprom.h` reference. Every Bluejay claim
with a `.asm` reference. The ESP32-S3 RMT budget of 4 TX and 4 RX channels,
confirmed against `soc_caps.h:285-287` in your local IDF v5.5.1.

Taken from documentation or forum posts and not confirmed against source. The
EDT frame encoding table, though the AM32 sender code agrees with it on the
voltage and current prefixes. The 24-128 kHz PWM frequency range. The claim
that a universal USB linker covers AM32 and BLHeli_S alike. The crawler wiki's
200 RPM guidance. Everything in the ESCape32 section. Bluejay shipping EDT in
v0.17 specifically, though the EDT code itself is in the tree.

Already found wrong and corrected. The wiki's EEPROM table, which is stale in
several places and understates the buffer size by a factor of four. The
description of "Running Brake Level" as a brake power percentage. The claim
that Bluejay's braking strength is adjustable in any useful sense. The idea
that ESC names can be stamped into bytes 5-16.

## What Bluejay already does, and where it actually stops

Worth pinning down, because the AM32 braking delta is narrower than it looks
from the AM32 docs.

**There is no brake input in Bluejay at all.** Damped light is always on, but
the braking duty is not a quantity anyone sets. It is derived from the drive
duty inside the Timer1 PWM ISR, `Modules/Isrs.asm:417-446`, by taking the
normal PWM value and subtracting dead time. The whole block sits inside
`IF DEADTIME != 0`, so on targets without dead time insertion it does not exist.

`DEFAULT_PGM_BRAKING_STRENGTH EQU 255 ; 0..255 => 0..100 % Braking` loads into
`Pwm_Braking_H/L` (`Modules/Settings.asm:167-197`) and does one thing, it caps
that derived value:

```asm
    subb A, Pwm_Braking_L
    mov  A, Temp5
    subb A, Pwm_Braking_H               ; Is braking pwm more than maximum allowed braking?
    jc   t1_int_pwm_braking_set         ; Yes - branch
    mov  Temp4, Pwm_Braking_L           ; No - set desired braking instead
```

At the default of 255 the clamp never fires, so out of the box the setting does
nothing. Lowering it reduces how hard the motor decelerates on throttle drop,
across the board.

**No DShot command reaches it.** The full command set Bluejay handles is in
`Modules/DShot.asm:79-215`, which is beeps, direction normal and reverse, bidir
off and on, extended telemetry enable and disable, temporary user direction
normal and reverse, and save settings. Nothing writes `Pgm_Braking_Strength`.
Changing it means the BLHeli bootloader over the signal wire with the motor
stopped, so once the car is moving the value is fixed. For what you want it is
not a usable control, and I should not have called it adjustable.

**Brake on stop is genuinely binary.** `DEFAULT_PGM_BRAKE_ON_STOP EQU 0 ;
1=Enabled 0=Disabled`, and the implementation is
`Bluejay.asm:1087-1093`:

```asm
    mov  Temp1, #Pgm_Brake_On_Stop      ; Check if using brake on stop
    mov  A, @Temp1
    jz   exit_run_mode_brake_done
    A_Com_Fet_On                        ; Brake on stop
    B_Com_Fet_On
    C_Com_Fet_On
```

Three low-side FETs on, hard short, no PWM, no strength. That's the FET lock you
hit, and there's no parameter that softens it. Dithering it at the frame rate
was the only lever available.

**3D mode already brakes before reversal.** `run6_bidir_braking`
(`Bluejay.asm:970-985`) sets the brake flag, clamps power to `Pwm_Limit_Beg`,
and holds until `Comm_Period4x_H` crosses `#40h` (~5000 eRPM) before flipping
`Flag_Motor_Dir_Rev`. It terminates at ~5000 eRPM rather than standstill, but it
overlaps a good part of what `tick_reversing()` is doing in software.

**The gap is known and not being closed.** bird-sanctuary/bluejay issue #221,
"Introducing Car Mode in BlueJay", opened June 2025, proposes exactly this:
remapping DShot values like 3D mode with dynamic brake power, for in-wheel-drive
RC cars doing torque vectoring, explicitly citing missing coast/freewheel and
proportional brake. Open, unlabelled, unassigned, no implementation.

The requirement here is commanding brake magnitude per control iteration, per
wheel, from the 500 Hz loop. Bluejay offers nothing that meets it. The only
influence you have over braking while driving is the throttle value itself,
since damped light decelerates harder the further the commanded throttle sits
below current speed, and that effect fades as the motor slows and is gone at
standstill.

So the deltas worth chasing are brake magnitude as a per-frame commanded value,
braking that still works at and near zero speed, and a choice between coast and
brake.

## Braking

AM32 has `proportionalBrake()` (PWM'd low-side short at the carrier frequency,
24-128 kHz, 2000 duty steps) and `fullBrake()` (hard short, what DShot 0 gives
you now). The firmware picks between them on a single threshold:

```c
if (prop_brake_duty_cycle >= 1999) { fullBrake(); } else { proportionalBrake(); }
```

Four distinct paths reach that code:

| Mechanism | Setting | Behaviour |
|---|---|---|
| Drag brake at standstill | `brake_on_stop = 1` + `drag_brake_strength` 1-10 | `prop_brake_duty_cycle = drag_brake_strength * 200` out of 2000. Ten proportional levels. |
| Active brake at standstill | `brake_on_stop = 2` + `active_brake_power` | Drives a commutation step to hold position. Requires armed. |
| Running brake | `comp_pwm = 1` + `driving_brake_strength` | Complementary PWM conducts the low-side FET during PWM off-time, so braking torque follows the duty drop and comes out of the throttle channel. `driving_brake_strength` limits it by widening dead time, see below. |
| Reverse-torque brake | `rc_car_reverse = 1` | Stick past neutral while rolling scales `prop_brake_duty_cycle` by distance past neutral. |

`rc_car_reverse` is the direct structural replacement for your dither
(`Src/main.c:1251-1265`):

```c
if (eepromBuffer.rc_car_reverse && prop_brake_active) {
    if (newinput <= 1047 && newinput > 47) prop_brake_duty_cycle = ((newinput - 48) * 2 + 47) - reversing_dead_band;
    if (newinput > 1047)                   prop_brake_duty_cycle = ((newinput - 1048) * 2 + 47) - reversing_dead_band;
    if (prop_brake_duty_cycle >= 1999) fullBrake(); else proportionalBrake();
}
```

Same concept you built, moved from a 500 Hz frame rate to the PWM carrier, with
2000 steps instead of 10.

### `rc_car_reverse` and sine startup are mutually exclusive

This is the finding that actually constrains the decision, and it isn't in any
of the documentation. `Src/main.c:1814`, comment is the firmware's own:

```c
if (eepromBuffer.rc_car_reverse) { // overrides a whole lot of things!
    throttle_max_at_low_rpm = 1000;
    eepromBuffer.bi_direction = 1;
    eepromBuffer.use_sine_start = 0;
    low_rpm_throttle_limit = 1;
    eepromBuffer.variable_pwm = 0;
    eepromBuffer.brake_on_zero_throttle = 0;
    eepromBuffer.comp_pwm = 0;
    eepromBuffer.stuck_rotor_protection = 0;
    minimum_duty_cycle += 50;
    stall_protect_minimum_duty += 50;
    min_startup_duty += 50;
}
```

And separately at `Src/main.c:1810`:

```c
if (!eepromBuffer.comp_pwm) { eepromBuffer.use_sine_start = 0; } // sine start requires complementary pwm
```

So the choice is:

**Config A — `rc_car_reverse = 1`.** ESC-side proportional brake-and-reverse off
the throttle channel. Loses sine startup, complementary PWM, variable PWM,
stuck-rotor protection. Minimum/startup duty cycles all bumped +50, so worse
crawl authority.

**Config B — `comp_pwm = 1`, `use_sine_start = 1`, `bi_direction = 1`.** Sine
startup, `driving_brake_strength` capping the decel, `brake_on_stop` +
`drag_brake_strength` for standstill. Reverse is the bidirectional throttle
split, so brake-before-reverse sequencing stays yours.

On the on-the-fly requirement, only Config A delivers. Its
`prop_brake_duty_cycle` is recomputed from the incoming DShot value on every
frame, so brake magnitude is a number you send at 500 Hz, and it holds down to
standstill. Config B's `driving_brake_strength` is an EEPROM byte, same shape of
limitation as Bluejay's `PGM_BRAKING_STRENGTH`. What Config B gives you is
throttle-proportional decel through complementary PWM, which is close to what
Bluejay's damped light already does. Config B is a modest improvement on your
current braking, not a change in kind.

Config A is also the better fit for torque vectoring, which reverses what I
said earlier. Every ESC has its own signal wire and its own DShot stream, and
`prop_brake_duty_cycle` is computed from that ESC's own `newinput`, so brake
magnitude is per wheel and independent across the four. That means you can
brake the inside wheels through a corner, not only decelerate in a straight
line. Config B's negative half is commanding less throttle than current speed
and letting damped light do what it does, which weakens as the wheel slows and
is gone at standstill.

A single signed value per wheel is the whole command surface either way. A
motor at any instant produces either accelerating or decelerating torque, so
there is nothing to separate. In Config A the negative half of that signed
value is brake torque you set. In Config B it is a side effect you do not
control. Both give independent wheels. They differ in what the negative half
is worth.

### The catch in Config A

Braking and driving cannot overlap, and switching between them costs a frame.
From `setInput()` at `Src/main.c:1053-1090`, when the commanded side opposes
the current `forward` direction the firmware sets `adjusted_input = 0` and
`prop_brake_active = 1`, so drive torque goes to zero for the whole time that
wheel is braking. The direction only flips once `return_to_center` is set, and
that is set in one place:

```c
if (newinput < 48) {
    adjusted_input = 0;
    if (prop_brake_active) {
        prop_brake_active = 0;
        return_to_center = 1;
    }
}
```

So going from braking a wheel to driving it the other way needs DShot below 48
for at least one frame. At 500 Hz that is a 2 ms transit, which is affordable,
but the controller has to know about it. There is no blending between brake and
drive on one wheel.

### Config A gives you coast, brake and drive as three distinct states

Yes, freewheeling works, and it is the neutral state rather than something you
configure.

`proportionalBrake()` in `Mcu/g071/Src/phaseouts.c:33` describes itself as
alternating "between braking (ABC LOW) and coasting (ABC float)". It turns all
three high FETs off and puts all three low FETs on the timer, so duty cycle is
the fraction of time the motor is shorted against the fraction it is floating.
That is the brake magnitude knob.

Coasting falls out of `comp_pwm = 0`, which Config A forces. In `phaseAPWM()`
at `phaseouts.c:137`, when `temp_comp_pwm` is zero the low FET is set to output
and driven off rather than handed to the timer:

```c
void phaseAPWM()
{
    if (!temp_comp_pwm) {
        LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW, LL_GPIO_MODE_OUTPUT);
        PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_A_GPIO_LOW;
    } else {
        LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_LOW, PHASE_A_GPIO_LOW, LL_GPIO_MODE_ALTERNATE);
    }
    ...
```

At neutral in Config A the commanded duty is zero, so the PWM phase has both
FETs off and `comStep` leaves a third phase floating. Two of three phases open
means no path for circulating current, so the wheel freewheels. Nothing calls
`allOff()` or either brake function on that path, because `allOff()` sits
behind `if (!running)` and `prop_brake_active` is clear at neutral.

So per wheel you get three states from one signed value. Command the current
direction and you drive. Command below 48 and you coast. Command the opposing
side and you brake, with magnitude set by how far past. This is what issue #221
is asking Bluejay for.

### `drag_brake_strength` is unreachable in both configs

Correcting the table above. The proportional standstill brake only exists on
the `comp_pwm` enabled branch. With `comp_pwm = 0`, which Config A forces,
`brake_on_stop` reaches `fullBrake()` and nothing else:

```c
if (!eepromBuffer.comp_pwm) {
    duty_cycle_setpoint = 0;
    if (!running) {
        ...
        if (eepromBuffer.brake_on_stop) {
            fullBrake();
        } else {
            if (!prop_brake_active) { allOff(); }
        }
    }
```

So standstill hold in Config A is the same hard short you already have on
Bluejay, all or nothing. Meanwhile Config B reaches the proportional version
but it is guarded by `if (!eepromBuffer.use_sine_start)` at `Src/main.c:1274`,
and Config B has sine startup on. `drag_brake_strength` needs `comp_pwm = 1`
together with `use_sine_start = 0`, which is neither config.

For Config A this matters less than it sounds. Holding the car stopped on a
slope wants the hard short anyway, and everything above zero speed is covered
by `proportionalBrake()`.

### What Config A actually costs

Losing `use_sine_start` is the whole of it that matters. The duty floors do go
up, `minimum_duty_cycle`, `stall_protect_minimum_duty` and `min_startup_duty`
each gain 50, so the smallest output the motor can produce is coarser. But
`throttle_max_at_low_rpm` goes from a default of 400 to 1000 out of 2000
(`Src/main.c:429` against `:1815`), which raises the low-RPM ceiling and gives
you more torque available down low. I previously described this as worse crawl
authority across the board, which was wrong. The floor rises and the ceiling
rises with it.

Note also `Src/main.c:1274`. The drag-brake block is guarded by
`if (!eepromBuffer.use_sine_start)`, so in Config B your standstill hold comes
from complementary PWM tracking rather than from `drag_brake_strength`.

### `brake_on_zero_throttle`

Newer setting, fires the moment throttle hits zero while still rolling
(`Src/main.c:1509-1525`). Source reads:

- `1` → `temp_comp_pwm = 0`, coast
- `2` → `temp_comp_pwm = 1`, brake
- `3..9` → brake after `value - 2` seconds

The 2.x release notes and several blogs have 1 and 2 swapped relative to this.
Check against whatever version ships on your ESC.

### Regen

Complementary PWM braking pushes energy back into the pack and AM32 has no brake
resistor. Four wheels decelerating a 1/10 car is enough to lift pack voltage
noticeably near full charge. You log `battery` at 10 Hz already, so it's visible
during the first braking tests — set LVC/overvoltage deliberately rather than
discovering it.

## EDT: telemetry over the wire you already have, on hardware you already have

Extended DShot Telemetry interleaves temperature, voltage, current, stress and
status into the bidirectional frames you're already decoding. No telemetry wire,
no UART, no software serial — the UART question doesn't arise.

**EDT originated in Bluejay and shipped in v0.17.0 (Oct 2022).** You're on
v0.21.x-era firmware, so this works on the ESCs currently in the car. Bluejay
even has `DEFAULT_PGM_SAFETY_ARM` gating it and a `Flag_Ext_Tele` in
`Bluejay.asm`. AM32 implements the same spec, so the decoder you write now
carries over unchanged.

This is the part of the investigation with the best return: it needs no
purchase, no config commitment, and it de-risks the only nontrivial ESP32-side
change in the whole AM32 migration.

Frame is `eeem mmmm mmmm cccc`. EDT reuses the encodings where bit 8 is zero and
the low nibble of the 12-bit payload is nonzero — patterns unreachable by a
valid eRPM. Type is the top nibble:

| Prefix | Type | Range |
|---|---|---|
| `0010` | Temperature | 0-255 °C |
| `0100` | Voltage | 0.25 V steps, 0-63.75 V |
| `0110` | Current | 1 A steps, 0-255 A |
| `1000` / `1010` | Debug | |
| `1100` | Stress level | 0-255 |
| `1110` | Status / version | alert, warning, error flags |

CRC is unchanged (`crc = (v ^ (v >> 4) ^ (v >> 8)) & 0x0F` over the 12 data
bits), so `gcr_to_erpm_data` stays as-is. Enable with DShot command 13 sent ≥6
consecutive times; ESC replies with a version frame. 14 disables.

Scheduling from `Src/dshot.c`:

```c
#define TEMP_EDT_RATE_DIVISOR    200
#define VOLTAGE_EDT_RATE_DIVISOR 200
#define CURRENT_EDT_RATE_DIVISOR 40
```

At 500 Hz that's current ≈12.5 Hz, voltage and temperature ≈2.5 Hz each, costing
~3% of eRPM samples.

ESP32 side: branch on frame type after CRC before calling
`erpm_data_to_erpm_period()` (which currently assumes every valid frame is
eRPM), hold last temp/voltage/current per motor, and add command 13 ×6 to the
arming sequence — `tick_commanding()` already has the repetition machinery at
`CMD_SEND_FRAMES = 10`. No new RMT channels, which matters since 4 × (TX+RX) is
exactly the S3's 4+4 budget and `rgb_led` is on SPI3.

If you ever wanted KISS serial anyway: UART0 is free
(`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`, `CONFIG_ESP_CONSOLE_UART_NUM=-1`,
`CONFIG_SOC_UART_NUM=3`, UART1 on RC, UART2 on GPS). One free UART against four
ESCs at 115200 one-wire each means RMT RX decoding or a mux, which is why EDT is
the answer rather than a fallback.

## Configuring all four ESCs from firmware

AM32 has an undocumented-ish DShot programming mode
(`Src/dshot.c:110-128`, `:224`) that writes EEPROM over the signal wire:

1. DShot command **36** → enter programming mode
2. next frame's 11-bit value = EEPROM byte index
3. next frame's value = new byte value
4. next frame's value = **37** → commit to the in-RAM EEPROM buffer
5. repeat 2-4 per setting
6. DShot command **12** (`SAVE_SETTINGS`) → persist to flash

Commands only process when `running == 0 && armed`, and need 6 consecutive
repeats (beacons 1-5 fire immediately). You already have `SAVE_SETTINGS = 12` in
`dshot_types.h` and a working command path since the telemetry-bit fix in
`docs/dshot_command_fix.md`.

This does not solve the on-the-fly problem. Commands only process when
`running == 0 && armed`, so you can rewrite `driving_brake_strength` between runs
but not while the wheels are turning. Treat it as boot-time configuration for
four ESCs from one place, which is worth having on its own, rather than as a
way to retune braking mid-drive. Flashing is otherwise unchanged, same
universal linker and web configurator you already use.

EEPROM layout, taken from `Inc/eeprom.h` rather than the wiki page, which is
stale. The buffer is 192 bytes, not the 48 the wiki states, and several of its
descriptions do not match the struct.

| Byte | Field | Notes |
|---|---|---|
| 5 | `max_ramp` | Throttle ramp limit, 0.1% per ms up to 25% per ms. Default 160. |
| 6 | `minimum_duty_cycle` | 0.2% to 51% |
| 7 | `disable_stick_calibration` | |
| 8 | `absolute_voltage_cutoff` | 1-100 in 0.5 V steps |
| 9-11 | `current_P` / `current_I` / `current_D` | 0-255 each |
| 12 | `active_brake_power` | 1-5 percent duty cycle |
| 13 | `brake_on_zero_throttle` | see above |
| 17 | `dir_reversed` | |
| 18 | `bi_direction` | |
| 19 | `use_sine_start` | |
| 20 | `comp_pwm` | |
| 21 | `variable_pwm` | |
| 22 | `stuck_rotor_protection` | |
| 23 | `advance_level` | 0-3 in the old format, 10-42 with config tools 1.90 and later. Converted on load at `Src/main.c:625-631`. Not the "×7.5 degrees" the wiki claims. |
| 24 | `pwm_frequency` | |
| 25 | `startup_power` | |
| 26 | `motor_kv` | |
| 27 | `motor_poles` | |
| 28 | `brake_on_stop` | 0=off, 1=drag, 2=active |
| 29 | `stall_protection` | |
| 30 | `beep_volume` | |
| 31 | `telemetry_on_interval` | |
| 32-35 | `servo.low_threshold` / `high_threshold` / `neutral` / `dead_band` | |
| 36 | `low_voltage_cut_off` | |
| 37 | `low_cell_volt_cutoff` | |
| 38 | `rc_car_reverse` | |
| 39 | `use_hall_sensors` | currently reused for auto timing advance |
| 40 | `sine_mode_changeover_thottle_level` | 5-25%, clamped at `Src/main.c:687` |
| 41 | `drag_brake_strength` | 1-10, out of range clamps to 10 at `Src/main.c:690` |
| 42 | `driving_brake_strength` | 1-10, see below |
| 43-44 | `limits.temperature` / `limits.current` | |
| 45 | `sine_mode_power` | 1-10, clamped at `Src/main.c:738` |
| 46 | `input_type` | |
| 47 | `auto_advance` | |
| 48-175 | `tune[128]` | startup tune |
| 176-183 | CAN settings | node id, esc index, telem rate, and so on |

There is no ESC name field in the struct, so the wiki's "ESC Name, 12 bytes at
5-16" is wrong and the idea of stamping FL/FR/RL/RR into the ESCs does not
work. Bytes 5-12 are live settings. I had repeated that suggestion from the
wiki without checking.

### What `driving_brake_strength` actually does

The configurator calls this "Running Brake Level", which is what the ArduPilot
thread and the wiki describe as applying a percentage of power to brake the
motor. That description is wrong. `Src/main.c:694-711`:

```c
if (eepromBuffer.driving_brake_strength == 0 || eepromBuffer.driving_brake_strength > 9) { // motor brake 1-9
    eepromBuffer.driving_brake_strength = 10;
}
if (eepromBuffer.driving_brake_strength < 10) {
    dead_time_override = DEAD_TIME + (150 - (eepromBuffer.driving_brake_strength * 10));
    if (dead_time_override > 200) { dead_time_override = 200; }
    min_startup_duty      += dead_time_override;
    minimum_duty_cycle    += dead_time_override;
    throttle_max_at_low_rpm += dead_time_override;
    startup_max_duty_cycle  += dead_time_override;
    TIM1->BDTR |= dead_time_override;   // STMICRO
```

It sets it once at boot by widening the PWM dead time, which reduces how long
the complementary FET conducts and therefore weakens braking. As a side effect
it raises all four duty thresholds, so a low value costs you low-speed
authority in exchange for softer braking. 10 means no override and full
braking. This is a boot-time hardware timing change, so the earlier conclusion
holds, it is not something you can command per frame.

It also explains issue #150, where setting the level to 1 makes the ESC appear
dead. Dead time goes to 190 and the minimum duty thresholds go up with it.

## Other capabilities

- **Sine startup**: open-loop sine below `sine_mode_changeover_throttle_level`
  (5-25% of range), torque set by `sine_mode_power` 1-10. The crawler wiki's
  practical test is whether the motor/ESC pair holds ~200 RPM in standard mode;
  below that, sine mode exit stalls. Drop `sine_mode_power` if the ESC heats
  while crawling.
- **Stall protection**: PID that boosts duty as RPM falls (`Src/main.c:1442`),
  source comment "this boosts throttle as the rpm gets lower, for crawlers".
  Note this interacts with any outer-loop torque control you write — two
  integrators on the same actuator.
- **Variable PWM** 24-128 kHz scheduled against commutation interval.
- **Timing advance** in 7.5° steps; auto timing advance currently rides on the
  "use hall sensors" switch as an experiment.
- **Firmware LVC**, per-cell or pack. Independent backstop to `power_monitor`.
- **DroneCAN** exists. Irrelevant here beyond signalling active maintenance.

## Repo impact (Config B)

| Area | Change |
|---|---|
| `Motor::brake()` | Delete the dither. Brake becomes negative throttle authority, or falls out of the throttle command via complementary PWM. |
| `BRAKE_DITHER_PERIOD`, `BRAKE_THROTTLE`, `State::BRAKING` | Gone. |
| `Motor::tick_reversing()` | Direction becomes a value, not a command sequence with a standstill wait. `DECEL_THROTTLE`, `STANDSTILL_TIMEOUT_MS`, `STANDSTILL_CONFIRM_FRAMES`, `ReversePhase` all go. |
| Throttle mapping | Bidirectional splits DShot into 48-1047 reverse / 1048-2047 forward — ~1000 steps per direction. |
| `DshotDriver` | EDT frame-type demux after CRC; per-motor temp/voltage/current. |
| `motor::RpmReading` | Extend, or add a sibling `EscTelemetry` topic. |
| `blackboard` | New ESC telemetry topic at its native 2.5-12.5 Hz, not per control iteration. `codebase-overview.md` already flags `rpm` at 500 Hz as 14 KB/s of the 40 KB/s total — worth rate-limiting in the same pass. |
| Arming | DShot 13 ×6 for EDT; optionally the programming-mode config push. |

## Open questions for the bench

- Whether the vehicle controller needs brake and drive as separate per-wheel
  outputs, or whether one signed torque per wheel covers it. This decides
  Config A versus Config B and should be answered before any hardware is
  committed.
- What `PGM_BRAKING_STRENGTH` below 255 feels like on the current Bluejay ESCs.
  This will not give you runtime control, so it is diagnostic only. It tells
  you whether the axles failed from the dither's 50 Hz shock loading or from
  braking magnitude, which is useful for sizing how much brake authority you
  can actually use once you have it.
- Whether Config B's low-speed authority beats Config A's on your specific
  motor/ESC pair. The 200 RPM standard-mode threshold is the test.
- Pack voltage rise under four-wheel regen at high SoC.
- Whether `stall_protection`'s duty boost fights the vehicle controller's own
  loop, and whether it's worth disabling even in Config B.
- Whether `brake_on_zero_throttle` numbering on your firmware version matches
  the source reading above.

## ESCape32

Same hardware class, generally migratable on AM32 crawler ESCs. Differentiator
is telemetry breadth — KISS, iBUS, S.Port, **CRSF**, plus EDT — and input
breadth including CRSF and SBUS directly. Given this project already speaks CRSF
to the transmitter, native CRSF telemetry out of the ESC is an interesting
shape. It lacks AM32's DShot programming mode, which is the thing that makes
four-ESC config tractable from firmware.

Not checked, and worth checking before committing to AM32: whether ESCape32
exposes brake magnitude as a commanded value rather than a stored setting. Its
stated design goals mention robust direction reversals, so it may have
something in this area. This was left open because the search budget ran out,
not because it looked unpromising.

## Sources

- [AM32 firmware repository](https://github.com/am32-firmware/AM32) — `Src/main.c`, `Src/dshot.c`
- [AM32 wiki: ESC Configuration Settings](https://wiki.am32.ca/guides/ESC-Settings-Explained.html)
- [AM32 wiki: Using AM32 with crawlers](https://wiki.am32.ca/general/Crawler-Hardware-and-AM32.html)
- [AM32 wiki: Open ESC EEPROM Format](https://wiki.am32.ca/development/Open-ESC-EEPROM-Format.html) — stale, contradicted by `Inc/eeprom.h`, kept here only to record where the bad table came from
- [AM32 `Inc/eeprom.h`](https://github.com/am32-firmware/AM32/blob/main/Inc/eeprom.h)
- [Extended DShot Telemetry specification](https://github.com/bird-sanctuary/extended-dshot-telemetry)
- [DShot — the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [Betaflight DShot documentation](https://betaflight.com/docs/development/API/Dshot)
- [ArduPilot: AM32 parameters, complementary PWM and running brake level](https://discuss.ardupilot.org/t/am32-parameters-complimentary-pwm-and-running-brake-level/132400)
- [AM32 issue #150: running brake level 1 disables ESC](https://github.com/am32-firmware/AM32/issues/150)
- [AM32 releases](https://github.com/am32-firmware/AM32/releases)
- [ESCape32 wiki](https://github.com/neoxic/ESCape32/wiki/Installation)
- [Bluejay firmware](https://github.com/bird-sanctuary/bluejay) — `src/Bluejay.asm`, `src/Modules/Isrs.asm`, `src/Modules/Settings.asm`, `src/Settings/BluejaySettings.asm`
- [Bluejay issue #221: Introducing Car Mode](https://github.com/bird-sanctuary/bluejay/issues/221)
