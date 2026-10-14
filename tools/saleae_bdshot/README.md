# Bidirectional DShot analyzer (Logic 2)

Decodes bidirectional (inverted, idle-high) DShot as sent to an ESC, including
the 3D / car-mode split range where 48-1047 is one direction and 1048-2047 the
other.

## Why it needs Async Serial underneath

Logic 2 high-level analyzers cannot read raw edges; they consume another
analyzer's output. DShot encodes each bit as a pulse width, so a UART sampling
once per bit cannot recover it. Running Async Serial at three times the DShot
bit rate oversamples each DShot bit three times, and the sample at index `3n`
lands mid-bit, which separates a short pulse from a long one.

## Setup

1. Analyzers → Add → **Async Serial**
   - Input Channel: the ESC signal wire
   - Bit Rate: **3x the DShot rate** — DShot600 → `1800000`, DShot300 → `900000`
   - Bits per Frame: **48**
   - Inverted: **unchecked** for bidirectional DShot (the line already idles
     high, so the falling edge at frame start is the UART start bit).
     Check it only for standard idle-low DShot.
   - Leave parity none, 1 stop bit, LSB first.
2. Analyzers → Add → **Bidirectional DShot**
   - Input Analyzer: the Async Serial above
   - Polarity: `Bidirectional (idle HIGH, inverted CRC)`
   - Throttle range: `3D / car mode` for an ESC with `bi_direction` or
     `rc_car_reverse` on, otherwise `Normal`

Async Serial also frames the ESC's telemetry bursts, which are not DShot. Those
are labelled `ESC telemetry response` rather than decoded as bogus throttle.
Their presence is diagnostic on its own: the ESC only answers once it has
locked onto the signal (`inputSet == 1` in AM32), and that happens before it
arms, so responses mean the signal was understood even if the ESC never arms.

## Output

- `THR` / `LOW` / `HIGH` — throttle. In split mode the label gives the half, the
  offset from that half's base, and the percentage of that half's 999 steps.
  A value equal to the base (48 or 1048) is zero torque, not idle-with-hold.
- `CMD` — DShot commands 0-47, named where known.
- `DShot 0` — stop / neutral.
- `ESC telemetry response` — the ESC answered in the turnaround window. Not
  decoded; see below.
- `crc=OK` or `crc=FAIL exp<x> got<y>`. A CRC failure outside the turnaround
  window is a genuine problem with the signal.

## What it does not do

The ESC's GCR telemetry response is sent at 5/4 of the DShot bit rate, so a
single Async Serial instance cannot sample both it and the outgoing frames
cleanly — at 3x the DShot rate a telemetry bit is only 2.4 samples wide.
Decoding it would need a second Async Serial at the telemetry rate feeding a
second analyzer instance. This analyzer decodes the controller-to-ESC direction
only; for eRPM and EDT, export the capture and decode offline.

AM32's EDT frames carry current, voltage, temperature, and a one-shot
enable/disable announcement (`Src/dshot.c:242-277`). There is no armed flag and
no stress-level frame, so telemetry cannot tell you whether the ESC armed.

## Verification

Checked against `session 3 …sal`, channel 1 (bidirectional DShot600), by
simulating Logic 2's Async Serial sampling from the raw transitions and
comparing against an independent edge decoder: 4638 of 4638 sampled frames
matched with no CRC failures, including both halves of the split range and the
EDT_ENABLE command frame.

Logic 2 packs a 48-bit frame big-endian over a value whose bit i is wire bit i,
so `data[5]` carries the first bits on the wire, not `data[0]`. Getting that
backwards decodes every frame to garbage with a failing CRC.

Standard idle-low mode shares the same code path but has not been checked
against a capture.
