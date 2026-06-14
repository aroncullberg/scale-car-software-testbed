# DShot Command Bug Report — `aroncullberg__EscDriver`

**Date**: 2026-02-14  
**Component**: `managed_components/aroncullberg__EscDriver` (DShotRMT)  
**Severity**: Commands completely non-functional

---

## Summary

DShot special commands (values 0-47) sent via `DShotRMT::sendCmd()` are silently ignored by ESCs because the **telemetry request bit is never set to `1`**. The DShot protocol requires this bit to be `1` for the ESC to interpret a packet as a command rather than a throttle value.

---

## Root Cause

### Bug #1 — Telemetry bit hardcoded to `false` (Critical)

The DShot 16-bit frame layout is:

```
[11 bits: throttle/command] [1 bit: telemetry] [4 bits: CRC]
```

For **commands 1-47**, the DShot spec requires the telemetry bit = `1`. This is how the ESC distinguishes "command 7 = set spin direction" from "throttle value 7 = basically zero".

The driver **never sets this bit for commands**:

```cpp
// Constructor — telemetry_req initialized false, never changed
throttle = {
    .throttle = 0,
    .telemetry_req = false,  // "separate telemetry wire is not supported"
};

// sendCmd() — calls send() without touching telemetry_req
void DShotRMT::sendCmd(uint16_t cmd) {
    if (cmd <= 48) {
        send(cmd);  // telemetry_req is still false!
    }
}

// send() — only sets throttle value, telemetry_req unchanged
void DShotRMT::send(uint16_t value) {
    throttle.throttle = value;
    // throttle.telemetry_req is STILL false
    modeTx();
    rmt_transmit(rmt_tx_channel, dshot_encoder, &throttle, ...);
}
```

The encoder faithfully encodes `telemetry = false` into the frame → ESC sees a low throttle value, not a command.

**Fix**: Set `throttle.telemetry_req = true` before sending commands 1-47, and restore it to `false` after.

---

### Bug #2 — Off-by-one in command range check (Minor)

```cpp
if (cmd <= 48)  // allows value 48 through
```

DShot commands are 0-47. Value 48 is `DSHOT_THROTTLE_MIN` — the first throttle value. Should be:

```cpp
if (cmd < 48)
```

---

### Bug #3 — No command repetition support (Design Gap)

The DShot spec requires most commands to be sent **≥10 times consecutively** for the ESC to accept them. This is critical for:
- `SPIN_DIRECTION_1/2` (7, 8)
- `MODE_3D_ON/OFF` (9, 10)
- `SAVE_SETTINGS` (12)

`sendCmd()` sends exactly once. The existing `sendTicks()` method (used for arming) could serve this purpose but is private and only used internally.

---

## Affected Commands

Every command 1-47 is broken. Notable ones that would be useful on this platform:

| Command | Value | Purpose |
|---|---|---|
| `BEEP_1..5` | 1-5 | Locator beeps |
| `SPIN_DIRECTION_1/2` | 7-8 | Motor direction |
| `MODE_3D_OFF` | 9 | Exit 3D mode |
| `MODE_3D_ON` | 10 | Enter 3D mode |
| `SAVE_SETTINGS` | 12 | Persist ESC config |
| `SPIN_DIRECTION_NORMAL` | 20 | Normal rotation |
| `SPIN_DIRECTION_REVERSED` | 21 | Reversed rotation |

---

## Fix in New Motor Component

The new `motor` component addresses all three issues:

```cpp
void Motor::send_command(DshotCommand cmd) {
    if (state_ != State::IDLE) {
        ESP_LOGW(TAG, "send_command while not idle (state=%d)", (int)state_);
    }
    // Set telemetry bit to 1 for commands — this is the critical fix
    dshot_.send_value(static_cast<uint16_t>(cmd), /*telemetry=*/true);
}
```

The low-level `DshotRmt::send_value()` accepts an explicit `telemetry` parameter rather than relying on stored state. For command repetition, the `Motor` class will provide a `send_command_repeated(cmd, count)` helper.
