# DShot Command Fix

**Date**: 2026-02-14
**Component**: `managed_components/aroncullberg__EscDriver`

## Problem

DShot special commands (values 0-47) were completely non-functional. Sending commands like beep, spin direction change, or 3D mode toggle had no effect — the ESC silently ignored them. This forced the use of 3D mode configured via BLHeli rather than runtime DShot commands, and made features like motor direction reversal via DShot impossible.

## Root Cause

Three bugs in `DShotRMT`, one critical and two minor.

### Bug 1 — Telemetry bit never set for commands (critical)

The DShot 16-bit frame layout is:

```
[11 bits: throttle/command] [1 bit: telemetry] [4 bits: CRC]
```

For values 1-47, the ESC uses the telemetry bit to distinguish commands from very low throttle values. When telemetry bit = 0, value 7 means "throttle at 7" (below minimum, ignored). When telemetry bit = 1, value 7 means "set spin direction 1".

The driver initialized `telemetry_req = false` in the constructor and never changed it:

```cpp
// Constructor
throttle = {
    .throttle = 0,
    .telemetry_req = false,  // never changed anywhere
};

// sendCmd — calls send() without touching telemetry_req
void DShotRMT::sendCmd(uint16_t cmd) {
    if (cmd <= 48) {
        send(cmd);  // telemetry_req still false
    }
}
```

The encoder at `dshot_rmt_encoder.c:36-39` faithfully encoded whatever `telemetry_req` was set to:

```c
static void make_dshot_frame(dshot_frame_t *frame, uint16_t throttle, bool telemetry, bool bidirectional) {
    frame->throttle = throttle;
    frame->telemetry = telemetry;  // false for commands — ESC ignores them
    // ... CRC computation ...
}
```

Every command was being sent as a sub-minimum throttle value, not as a command.

### Bug 2 — Off-by-one in command range check (minor)

```cpp
if (cmd <= 48)  // allows value 48 through
```

DShot commands are 0-47. Value 48 is `DSHOT_THROTTLE_MIN`, the first valid throttle value. Should be `cmd < 48`.

### Bug 3 — No command repetition (design gap)

The DShot protocol requires most commands to be sent at least 10 consecutive times for the ESC to accept them. `sendCmd()` sent exactly once. The existing `sendTicks()` method (used internally for arming) already had the repeated-send pattern but was private and not exposed for commands.

## Fix

### `sendCmd()` — set telemetry bit around the send

```cpp
void DShotRMT::sendCmd(uint16_t cmd) {
    if (cmd < 48) {
        throttle.telemetry_req = true;
        send(cmd);
        throttle.telemetry_req = false;
    }
}
```

Sets `telemetry_req = true` before sending so the encoder produces a valid command frame, then restores it to `false` so subsequent throttle sends aren't affected. Also fixes the off-by-one (`< 48` instead of `<= 48`).

### `sendCmdRepeat()` — new public method for repeated commands

```cpp
void DShotRMT::sendCmdRepeat(uint16_t cmd, uint16_t count, TickType_t interval) {
    if (cmd >= 48)
        return;

    for (uint16_t i = 0; i < count; i++) {
        sendCmd(cmd);
        if (i < count - 1)
            vTaskDelay(interval);
    }
}
```

Sends a command `count` times with a configurable interval (default 1 tick) between sends. This satisfies the DShot spec requirement for commands like `SPIN_DIRECTION` (7/8), `MODE_3D_ON/OFF` (9/10), and `SAVE_SETTINGS` (12) which all need 10+ repetitions.

## Verification

Bench tested with a single ESC on GPIO 41 (motor_fl):

- **Beep commands (1-5)**: all five beep tones audible and distinct — confirms the telemetry bit fix works and the ESC recognizes commands.
- **Direction commands (7/8)**: motor spins in opposite directions after `SPIN_DIRECTION_1` vs `SPIN_DIRECTION_2`, confirming repeated command sending works.

## Impact

With commands working, the system can now:
- **Switch motor direction at runtime** via DShot commands 7/8 (temporary) or 20/21 (persistent), eliminating the need for 3D mode to achieve reverse.
- **Run in normal mode** with full 48-2047 throttle range (2000 steps) instead of 3D mode's 1000 steps per direction — doubling throttle resolution.
- **Send beep commands** for locating the vehicle.
- **Configure ESC settings** (3D mode on/off, save settings) programmatically without BLHeli.

## Files Changed

| File | Change |
|---|---|
| `managed_components/aroncullberg__EscDriver/DShotRMT.cpp` | Fixed `sendCmd()` to set telemetry bit, fixed off-by-one, added `sendCmdRepeat()` |
| `managed_components/aroncullberg__EscDriver/DShotRMT.h` | Added `sendCmdRepeat()` declaration |
