# DShot generation: AM32 migration (WIP, non-functional)

The motor component was rewritten to drive AM32 ESCs with `rc_car_reverse = 1`
instead of Bluejay. The ESC does not arm in the configuration this targets. This
records what the signal path does now and what changed, so the next session does
not have to rediscover it.

There is no committed baseline for `dshot_driver.cpp` or `motor.cpp` — they were
untracked before this work, so the previously working Bluejay behaviour cannot be
diffed. The closest reference is `managed_components/aroncullberg__EscDriver/`,
the library this driver was ported from.

## Bench results

| `inverted` | `telemetry` | Result |
|---|---|---|
| false | false | ESC arms, motor spins under throttle |
| true | false | Power-on tone only, never arms |
| true | true | Power-on tone only, never arms, no eRPM ever returned |

The ESC runs bidirectional DShot correctly when driven from Betaflight, and from
a known-good FC it plays the arm tone. Signal continuity and shared ground were
verified on the bench.

AM32 plays `playStartupTune()` unconditionally at boot, so the power-on tone
proves nothing about the signal. The tone that is missing is `playInputTune()`,
which fires only on the transition to `armed = 1` (`Src/main.c:1349-1370`). That
sits behind `inputSet` → `zero_input_count > 30` → one second of
`adjusted_input == 0`. `inputSet` is set by `detectInput()`/`checkDshot()`
(`Src/signal.c:203-229`), which measures edge intervals against fixed timer-tick
windows. Nothing downstream of it runs until it locks, including AM32's own
inverted-signal detection in `computeDshotDMA()`.

## Signal path

Unchanged from the working Bluejay setup:

- `dshot_rmt_encoder.c` builds the 16-bit frame and the RMT symbols. Not modified
  in this migration. Its `bidirectional` flag inverts the CRC.
- DShot600, `post_delay_us = 3`, RMT resolution 40 MHz.
- 500 Hz frame rate from the per-motor task (`pdMS_TO_TICKS(2)`, FreeRTOS at
  1000 Hz).
- Half-duplex switching: `mode_tx()`/`mode_rx()` with `gpio_ll_od_enable` during
  the receive window, RX armed from `tx_done_isr`. This is a faithful port of
  `DShotRMT::modeTx`/`modeRx`/`enableRx`/`disableRx`.

## What changed

**`bidirectional` split into `inverted` and `telemetry`.** They were one flag.
`inverted` drives both `.invert_out` on the TX channel and the encoder's CRC
inversion, since those are the same protocol variant. `telemetry` gates the RX
channel, the `tx_done` handoff and the open-drain window. This exists to separate
the two when bisecting; production wants both true.

**Values on the wire.** AM32 car mode splits the range into two 1000-step halves
whose base value is that half's zero point. Idle is no longer DShot 48.

| Intent | Before (Bluejay) | Now (AM32 car mode) |
|---|---|---|
| arm | `send_command(0)`, 3200 ms | unchanged |
| idle / coast | `send_throttle(48)` | drive half base, 1048 or 48 |
| drive | 48 + throttle × 1999 | half base + 1..999 |
| brake | dither between 0 and 148 at 50 Hz | opposing half base + 1..999 |
| reverse | DShot cmd 7/8 plus standstill wait | three-phase sync through a sub-48 frame |

**Direction handling.** DShot commands 7/8/20/21 are no longer sent. Which half
of the range drives a wheel is chosen host-side from `Mounting` and `Gear`.
Changing gear requires one frame below 48 while braking to set AM32's
`return_to_center`, which is what `tick_syncing()` does in three repeated phases.

**eRPM reporting.** `erpm()` returns `ErpmSample` with a three-way `RpmStatus`
(`Valid` / `Stopped` / `NoData`) instead of folding AM32's 0x0fff stopped code
and a CRC failure both to zero. The RX ISR now stamps `esp_timer_get_time()`, and
a decode older than `TELEMETRY_TIMEOUT_US` reads as `NoData`. Previously a silent
ESC left the last decoded value looking current indefinitely.

**Pull-up.** `gpio_pullup_en(gpio_)` is asserted after TX channel creation when
`telemetry` is on. `rmt_new_rx_channel` enables it (`rmt_rx.c:305`) but the TX
channel is created afterwards and re-runs `gpio_func_sel`. Whether that clears it
was not confirmed.

**Diagnostic counters.** `DshotDriver::RxStats` counts RX completions, symbols,
decode failures and RX-arm failures. Bench only, nothing depends on them.

## Open questions

Whether AM32's `detectInput()` can lock onto our inverted waveform at all. The
`inverted=true, telemetry=false` case sends idle-high DShot with inverted CRC,
driven push-pull with no open-drain window, and still does not arm. That points
away from the receive path and at the waveform itself, but it has not been
confirmed on a scope.

Whether DSHOT300 changes the outcome. Not tried on the current build. Wider
pulses give more margin against `checkDshot()`'s tick windows. One line in
`dshot_driver.cpp`.

Whether the ESC's `input_type` (EEPROM byte 46) is `DSHOT_IN` or `AUTO_IN`. If it
is `EDTARM_IN`, `EDT_ARMED` starts at 0 and throttle is ignored until DShot
command 13 arrives six times (`Src/dshot.c:129-135`).

Whether `rmt_enable`/`rmt_disable`/`rmt_receive` being called from `tx_done_isr`
matters. None are ISR-safe by contract. `CONFIG_PM_ENABLE` is off so the pm-lock
path is skipped, and `CONFIG_RMT_RECV_FUNC_IN_IRAM` is off so `rmt_receive` is a
flash fetch inside the interrupt. The reference library does the same thing, so
this is not a regression, but it is unverified on AM32's ~30 µs response window.

## Coupling to the vehicle controller

`control_motors()` gates gear changes on `stopped_evidence`, which needs
`RpmStatus::Stopped` from at least one wheel. With `telemetry` off, `rpm()` is
permanently `NoData` and gear changes never fire. Drive and brake are unaffected.
A time-based standstill fallback would decouple the two if the receive path stays
broken.
