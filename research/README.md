# research

Working notes and investigations. Not authoritative, dated at the top of each
file, and expected to go stale.

- [codebase-overview.md](codebase-overview.md) — what the firmware is, the
  component map, task rates, measured blackboard record sizes and data rate, and
  the real-time constraints anything new has to live with.
- [logging-alternatives.md](logging-alternatives.md) — where recorded bytes
  should live. Three options compared. Settled: SD card over SDMMC.
- [logging-design.md](logging-design.md) — how the logging itself should be
  structured now that storage is settled. Critique of the current blackboard
  record design, the push/pull question, and a self-describing format proposal.
- [sd-card-wiring.md](sd-card-wiring.md) — SD mode vs SPI mode, bus width, the
  pin budget on this board, and the strapping-pin trap.
- [offload-triggers.md](offload-triggers.md) — how to get logs off the car and
  what carries the bytes.
- [am32-esc.md](am32-esc.md) — AM32 firmware behaviour as it applies to this
  vehicle's DShot setup.
- [dshot-pad-damage.md](dshot-pad-damage.md) — why GPIO41's input died while
  its output kept working, how the bidirectional DShot handover lets the ESP32
  and the ESC drive the same wire at once, and what GPIO18 needs to avoid it.
- [usb-msc-offload.md](usb-msc-offload.md) — exposing the SD card as a USB drive
  on the board's second USB port, while flashing stays on COM.
