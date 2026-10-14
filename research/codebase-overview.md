# Codebase Overview

Findings from reading the repo on 2026-08-22, at commit `997b970` plus a large
uncommitted working tree (the blackboard refactor).

## What the project is

An ESP32-S3 firmware for a 1/10 scale RC car that sits between the driver's
transmitter and the hardware. It reads RC input, IMU, GPS and battery, then
drives four brushless motors over DShot and four servos over MCPWM. Long term
goals in the README are torque vectoring and GPS waypoint navigation.

## Build and target

- ESP-IDF 5.5.1, target `esp32s3`, C++20, `-fno-rtti -fno-exceptions` in `main`.
- 16 MB flash, 8 MB octal PSRAM (N16R8). PSRAM is enabled and running at 80 MHz
  in octal mode (`CONFIG_SPIRAM_MODE_OCT`, `CONFIG_SPIRAM_SPEED_80M`). The
  design doc still says PSRAM is disabled, which is now out of date.
- CPU at 240 MHz, `FREERTOS_HZ=1000`, so a tick is 1 ms.
- Custom partition table in `partitions.csv`. About 3.2 MB of the 16 MB flash is
  claimed, leaving roughly 12.9 MB unallocated.
- Managed components: `esp32_BNO08x`, `ESP_CRSF` (the user's fork),
  `TinyGPSPlus-ESPIDF`, `led_strip`. There is also an unused
  `aroncullberg__EscDriver` directory that is not in `dependencies.lock`.
- The user builds manually. Claude does not run `idf.py build`.

## Component map

| Component | Role |
|---|---|
| `blackboard` | System-wide data bus. Seqlock latest-value per type plus one global tagged log ring. |
| `logger` | Drains the blackboard ring on a task and hands records to a `Sink`. |
| `tcp_sink` | A `Sink` that streams records over SoftAP + TCP with a 4 MB PSRAM store-and-forward ring. |
| `imu` | Facade + `BNO08xBackend` over SPI2. Pure producer, publishes accel, gyro, quat, euler. |
| `rc` | `rclink::Receiver` with SBUS and ELRS/CRSF backends, auto-detected. Also the CRSF downlink. |
| `nav` + `proto_nmea` | GPS types and the NMEA UART driver, publishes `nav::GpsFix`. |
| `motor` | DShot over RMT with a custom encoder, bidirectional, RPM feedback. |
| `servo` | MCPWM servo driver with grouping and failsafe. |
| `power_monitor` | ADC voltage and current sampling, publishes `PowerMonitor::Reading`. |
| `vehicle` | The 500 Hz control task. Software differential, PID steering, countersteer, arming. |
| `rgb_led` | WS2812 strip on GPIO48 over SPI3. |
| `waypoint_storage` | Waypoint persistence, not wired into `main` yet. |
| `shared` | Common sensor types. |

The `telemetry` component has been deleted in the working tree and replaced by
`blackboard`. The name telemetry now means only the CRSF downlink to the
transmitter.

## The blackboard, in short

`publish<T>(v)` does two independent things:

1. Stores `v` into a per-type seqlock `Snapshot<T>`. Single writer, many
   readers, readers copy and never block. This is what the control loop and the
   CRSF downlink read through `latest<T>()`.
2. Appends a tagged record to one global `RingbufHandle_t`
   (`RINGBUF_TYPE_NOSPLIT`, 16 KB) with a 0-tick acquire. A full ring drops the
   record and bumps `dropped_records()`. Producers never block on logging.

A type is bound to a topic id by specialising `blackboard::Topic<T>` next to the
struct in the owning component's header. The trait is declared but not defined,
so publishing an unbound type fails to compile. The blackboard core names no
producer type.

Record layout is `[topic_id:1][payload_len:1][timestamp_us:6][payload:N]`, so an
8 byte header and a raw struct payload. The timestamp is stamped inside
`ring_append` from `esp_timer_get_time()`, so every record shares one clock.

Design and rationale live in `docs/blackboard.md`. `docs/blackboard.md` claims
steps 1 to 4 done and step 5 (TcpSink) in progress. That matches the code.

## Task and rate inventory

| Task | Prio | Core | Period |
|---|---|---|---|
| `vehicle_control` | see `vehicle_controller.cpp` | pinned | 2 ms (500 Hz) |
| motor DShot loop | | pinned | 2 ms (500 Hz) |
| `ExpressLRS task` | | pinned | 10 ms (100 Hz) |
| NMEA driver | 5 | | 20 ms |
| power monitor | | | 100 ms |
| `logger` | 2 | any | blocks on the ring, 100 ms timeout |
| `tcp_srv` | 4 | any | drains the PSRAM ring to the socket |

IMU rates come from `imu_backend.start(cfg, 5000, 5000, 40000)`, meaning
5000 us for gyro and accel (200 Hz each) and 40000 us for the rotation vector
(25 Hz, which yields both quat and euler).

## Measured record sizes and data rate

Payload sizes computed from the actual struct definitions (`uint64_t` is 8 byte
aligned on both the host and Xtensa, so these match the target).

| Topic | Payload | Record | On-ring | Rate | Bytes/s |
|---|---|---|---|---|---|
| accel | 32 | 40 | 48 | 200 Hz | 8000 |
| gyro | 32 | 40 | 48 | 200 Hz | 8000 |
| quat | 40 | 48 | 56 | 25 Hz | 1200 |
| euler | 40 | 48 | 56 | 25 Hz | 1200 |
| rc_channels | 64 | 72 | 80 | 100 Hz | 7200 |
| rpm | 20 | 28 | 36 | 500 Hz | 14000 |
| battery | 16 | 24 | 32 | 10 Hz | 240 |
| gps | 32 | 40 | 48 | 10 Hz | 400 |

"On-ring" adds the `RINGBUF_TYPE_NOSPLIT` 8 byte item header and 4 byte
alignment.

**Total sink bandwidth is about 40 KB/s, or 2.4 MB per minute, or 141 MB per
hour.** Ring occupancy is about 49 KB/s, so the 16 KB ring holds roughly 330 ms
of backlog if the logger task stalls.

Two observations worth acting on:

- `rpm` at 500 Hz is the single largest contributor (14 KB/s of 40) and it is
  published from `control_motors` on every control iteration even though the
  ESC telemetry behind it does not update anywhere near that fast. Rate limiting
  it is the cheapest possible bandwidth win.
- `RcChannels` carries both `raw` and `scaled` for all 16 channels. Half of that
  is derivable offline.

## Things that look off

- `main.cpp` still contains a commented-out `logger_task` block from the old
  telemetry design.
- `blackboard::init()` has no failure path visible to callers. If
  `xRingbufferCreate` fails, every later `publish` silently increments the drop
  counter and nothing reports it. Nothing in the firmware ever reads
  `dropped_records()`.
- `logger::s_sink` is a plain pointer written from `start()` and read from the
  task with no synchronisation. Fine today because it is set once before the
  task exists, but it forbids swapping sinks at runtime.
- `topic_ids.h` reserves `link_stats` (4) and `flight_mode` (7) with no
  producer. Documented as deliberate.
- The record header has no schema version, so a struct layout change silently
  invalidates old logs. `docs/blackboard.md` calls this out and accepts it.
- There is no record framing beyond length, so a decoder that joins a byte
  stream mid-record cannot resynchronise. `tcp_sink` sidesteps this by dropping
  at record granularity, but a file-backed sink that gets truncated by a power
  cut will need a sync word.

## Real-time constraints that matter for any storage work

None of the following are set in `sdkconfig`, and all four are relevant the
moment anything writes to internal flash:

```
# CONFIG_RMT_ISR_IRAM_SAFE is not set
# CONFIG_UART_ISR_IN_IRAM is not set
# CONFIG_GPTIMER_ISR_IRAM_SAFE is not set
# CONFIG_SPI_FLASH_AUTO_SUSPEND is not set
```

DShot output runs on RMT without DMA (`with_dma = false`) and without an
IRAM-safe ISR. The RC and GPS UART ISRs are not in IRAM either. During an
internal flash write or erase the cache is disabled, non-IRAM ISRs do not run,
and any task executing from flash stalls. That is a direct hazard for the 2 ms
control loop and the DShot frame timing. This is the single most important
finding for the logging question and it is developed in
`logging-alternatives.md`.
