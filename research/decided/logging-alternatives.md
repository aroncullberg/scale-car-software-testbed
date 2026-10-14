# Logging: three alternatives

Research for the question "how should this system record everything it produces
so it can be analysed offline". Offloading the resulting data is explicitly out
of scope, but each option is judged partly on how painful offload would be,
since a format nobody can get at is not a log.

Written 2026-08-22. Numbers come from the code as it stands, see
`codebase-overview.md` for how they were derived.

## The numbers this has to satisfy

| Quantity | Value |
|---|---|
| Sink bandwidth at current rates | ~40 KB/s (2.4 MB/min, 141 MB/h) |
| Largest single contributor | `rpm` at 500 Hz, 14 KB/s |
| Blackboard ring | 16 KB, about 330 ms of backlog |
| Free flash after the current partition table | ~12.9 MB of 16 MB |
| PSRAM | 8 MB total, 4 MB already claimed by `tcp_sink` |
| Hard real-time deadline | 2 ms, the vehicle control loop and the DShot frame |
| Typical session length worth capturing | minutes, not seconds |

The first thing that falls out of this table: **12.9 MB of free flash at 40 KB/s
is 5.3 minutes of recording.** Any internal-flash plan is a short-session plan
unless the record rate comes down, and the whole point of the blackboard was to
capture everything.

## The constraint that decides most of this

The idea "log to a filesystem" is really the idea "write to internal SPI flash",
and internal flash writes on this chip are not free in the way an SD card write
is. Espressif's own documentation is blunt about it. With
`CONFIG_SPI_FLASH_AUTO_SUSPEND` disabled, which is the state of this
`sdkconfig`, a flash write or erase disables the caches, which
"suspends all the other tasks" and disables "all non-IRAM-safe interrupts" until
the operation completes.

That matters here specifically because of what this firmware looks like:

```
# CONFIG_RMT_ISR_IRAM_SAFE is not set      <- DShot output
# CONFIG_UART_ISR_IN_IRAM is not set       <- RC link and GPS
# CONFIG_GPTIMER_ISR_IRAM_SAFE is not set
# CONFIG_SPI_FLASH_AUTO_SUSPEND is not set
```

DShot runs on RMT with `with_dma = false`, so the frame is fed by an ISR that
will not run while the cache is off. The control task runs from flash at 500 Hz,
so it stalls too. The relevant config in this project is

```
CONFIG_SPI_FLASH_ERASE_YIELD_DURATION_MS=20
CONFIG_SPI_FLASH_ERASE_YIELD_TICKS=1
CONFIG_SPI_FLASH_WRITE_CHUNK_SIZE=8192
```

A multi-sector erase yields for one tick every 20 ms, so the system gets 1 ms of
air per 20 ms cache-off window. A single 4 KB sector erase cannot be yielded at
all and takes tens of milliseconds on typical NOR parts. **A 20 ms stall is ten
missed control iterations and ten missed DShot frames.** On a car under throttle
that is not a logging artifact, it is a control failure.

Everything below is shaped by how it handles that.

---

## Option A: raw append-only ring on a dedicated flash partition

Add a `log` partition covering the free ~12.9 MB, no filesystem at all, and
write records into it as a circular buffer with `esp_partition_write` and
`esp_partition_erase_range`.

**Shape**

```
blackboard ring (16 KB SRAM)
      -> logger task
      -> FlashSink: PSRAM staging buffer (1-2 MB)
      -> flusher task: writes 4 KB aligned blocks into the log partition
```

The staging buffer in PSRAM is not optional. It is what lets the writer batch
into whole sectors and, more importantly, what absorbs the stall while a write
is in progress.

**Why raw instead of a filesystem.** Sequential append is the only access
pattern here. A filesystem exists to give you naming, allocation and
crash-consistent metadata, and this workload wants none of those. Skipping it
removes the metadata write amplification, removes the copy-on-write behaviour
that makes LittleFS appends expensive, and removes an entire failure mode where
a power cut mid-metadata-update costs the whole log rather than the last sector.

**Erase policy is the whole design.** Never erase in the write path. Erase the
next N sectors ahead of the write head, and do it only when the vehicle is
disarmed, or before arming, or during a known idle window. If the write head
catches an unerased sector while armed, drop records and count the drops rather
than erase. This is what converts an unbounded 20 ms hazard into a bounded,
scheduled one.

**Endurance.** 12.9 MB at 40 KB/s wraps every 5.3 minutes and each sector is
erased once per wrap. At a nominal 100k erase cycles per sector, that is on the
order of 8000 hours of continuous recording before wear matters. Endurance is
not the limiting factor. Capacity is.

**Offload.** Since there is no filesystem, offload means reading the partition
back. Easiest path is a serial or WiFi command that streams the partition to a
laptop, reusing the existing `tcp_sink` transport. A second option is exposing
the partition over USB MSC. Neither is hard, but neither is free either, and
`esptool.py read_flash` is always the fallback.

**Verdict.** The best of the "keep it on the board" options. Cheap, no new
hardware, and the failure modes are all understandable. Capped at about five
minutes of full-rate history, which makes it a black box recorder rather than a
session recorder.

---

## Option B: a file on a LittleFS or FATFS partition

The literal version of the original idea. Mount a filesystem on the free flash
and append records to `run_0001.bin`.

**What the filesystem buys.** Named sessions, multiple runs on the device at
once, an obvious offload story (open the file, send it), and a format anyone can
reason about. LittleFS is already available as `joltwallet/littlefs` in the
component registry, and FATFS is already compiled into this build for other
reasons.

**What it costs.** All of Option A's cache-disable hazard, plus write
amplification, plus loss of control over when erases happen. That last part is
the killer. The filesystem decides when to allocate a block, when to rewrite
metadata, and when to garbage collect, and it does so inside your `write()` call,
on whatever task called it. You cannot schedule those erases into a safe window
because you do not know when they are coming.

The reported numbers are not encouraging either. FATFS with wear levelling has
been measured at under 250 KB/s for reads, with roughly 60 KB of log writes
taking about 9 seconds in one report. LittleFS append is structurally expensive
because its crash-safety model copies before appending. Community benchmarks put
LittleFS and FATFS in the same rough band for bulk writes, with LittleFS much
better at small-file operations and deletion. Even the optimistic end of that
range is only a few times the required 40 KB/s, which is a thin margin for a
system that also has to fly a control loop.

**If you do it anyway,** the mitigations are the same as Option A and then some.
Stage in PSRAM, write in large aligned chunks, preallocate the file to full size
at session start so the allocator does its work up front, never call `fsync` on
the hot path, and expect to lose the tail of the file on a power cut.

**Verdict.** Highest convenience, worst real-time behaviour, and the same 5.3
minute capacity ceiling as Option A. The convenience is real but it is bought
with the one resource this system cannot spend.

---

## Option C: SD card over SDMMC, with PSRAM staging

Wire an SD card to the SDMMC peripheral in 4-bit mode and write FATFS files to
it, fronted by a PSRAM staging buffer.

**Why this changes the shape of the problem.** SDMMC is a separate peripheral on
separate pins. Writing to an SD card does not touch SPI1, does not disable the
cache, and does not stop the control loop or the DShot ISR. The entire
cache-disable hazard that dominates Options A and B simply does not exist. What
remains is ordinary FreeRTOS scheduling, which is a problem this codebase
already knows how to solve.

**Capacity stops being a constraint.** A 32 GB card at 40 KB/s is roughly nine
days of continuous recording. Sessions can be hours. Full-rate IMU, unpruned RC
channels, and any future signal all fit without a conversation about budget.

**Throughput.** Reported ESP32-S3 numbers are about 2.34 MB/s in 4-bit SDIO mode
at 40 MHz versus about 1.01 MB/s in 1-bit SPI mode at 20 MHz, with some users
reporting far worse on particular cards. Even the pessimistic end is an order of
magnitude above the 40 KB/s needed.

**The real cost is hardware and pins.** SDMMC 4-bit needs CLK, CMD and D0 to D3,
so six pins including power, or three signal pins for 1-bit mode. On this board
GPIOs 2, 4, 6, 7, 9 to 14, 16, 17, 38 to 42, 47 and 48 are taken by peripherals,
1 and 15 by the ADC channels, and 26 to 37 are reserved by octal flash and
PSRAM. That leaves roughly GPIO 3, 5, 8, 18, 21, 43, 44, 45 and 46, and 45 and
46 are strapping pins that need care. Enough for 4-bit SDMMC, but not
comfortably, and it needs a board revision or flying leads.

**The other real cost is latency spikes.** SD cards stall for tens of
milliseconds during internal wear levelling and block erase, unpredictably. The
difference from Options A and B is that this stall blocks only the writer task,
not the CPU. A PSRAM staging buffer of a few megabytes covers it completely.

**Offload.** Pull the card. This is by a wide margin the best offload story,
which matters since offload is out of scope precisely because nobody wants to
build it.

**Verdict.** The only option that removes the constraint rather than working
around it, and the only one where the capacity matches the stated ambition of
capturing everything for offline modelling. It costs a board change.

---

## Comparison

| | A: raw flash ring | B: filesystem on flash | C: SD over SDMMC |
|---|---|---|---|
| New hardware | none | none | SD socket, 3 to 6 pins |
| Capacity at 40 KB/s | ~5.3 min | ~5 min | days |
| Stalls the control loop | yes, but schedulable | yes, unschedulable | no |
| Write amplification | none | moderate to high | low |
| Crash tolerance | loses last sector | can lose the file | loses last buffer |
| Offload effort | build a reader | open the file | pull the card |
| Implementation size | one sink, ~300 lines | one sink, less logic | one sink plus wiring |

## Recommendation

Do C for the real thing and A as the thing that ships this week.

They are not exclusive and they share almost everything. Both are a
`logger::Sink` behind a PSRAM staging buffer, which is the same structure
`tcp_sink` already has. Building the staging buffer once as a reusable piece and
putting either a flash writer or an SD writer behind it means the choice is a
swap, not a rewrite.

Concretely:

1. Factor the PSRAM store-and-forward ring out of `tcp_sink` into something both
   sinks share. It is already written and already debugged.
2. Add `FlashSink` (Option A) against a new `log` partition, with erase strictly
   confined to the disarmed state. This gives a five minute black box with no
   hardware change and no risk to the control loop.
3. Plan the SD card into the next board revision and add `SdCardSink` behind the
   same staging buffer. `docs/blackboard.md` already lists this as the intended
   endpoint.

Do not do B. It is the option that costs the most in real-time behaviour and
buys the least that A does not already provide.

## Work that pays off under all three options

These are orthogonal to the choice above and worth doing regardless.

- **Rate limit `rpm`.** 14 KB/s of the 40 is one topic published every control
  iteration at 500 Hz. The underlying ESC telemetry does not change that fast.
  Publishing it at 50 Hz cuts total bandwidth by nearly a third and buys 50%
  more recording time on any flash-backed option.
- **Trim `RcChannels`.** 64 bytes carrying both raw and scaled values for all 16
  channels, at 100 Hz. Scaled is derivable from raw offline.
- **Add a sync word and a schema version to the record header.** Currently a
  decoder cannot resynchronise on a truncated stream, and a struct layout change
  silently invalidates old logs. A power cut on a file-backed sink guarantees a
  truncated stream, so this stops being theoretical the moment anything writes to
  storage. `docs/blackboard.md` acknowledges the schema version gap and defers it.
- **Surface `dropped_records()`.** Nothing in the firmware reads it. A log with
  silent gaps is worse than a log with known gaps, and every option above drops
  records under pressure by design.
- **Consider delta or varint encoding in the sink.** Most fields change slowly
  between consecutive samples of the same topic. This is where a 2x to 4x
  reduction lives if capacity ever becomes the binding constraint again.

## Sources

- [Concurrency Constraints for Flash on SPI1, ESP32-S3, ESP-IDF v5.5](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/spi_flash/spi_flash_concurrency.html)
- [SPI Flash API, ESP32-S3, ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/spi_flash/index.html)
- [SDMMC Host Driver, ESP32-S3, ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32s3/api-reference/peripherals/sdmmc_host.html)
- [About esp32-s3 sdmmc write speed, espressif/esp-idf issue 11628](https://github.com/espressif/esp-idf/issues/11628)
- [ESP32 SD Card Optimization, Dror Gluska](https://blog.drorgluska.com/2022/06/esp32-sd-card-optimization.html)
- [Wear levelling FATFS performance really slow even for reading, ESP32 Forum](https://esp32.com/viewtopic.php?t=10267)
- [SPIFFS vs FATFS write speed, ESP32 Forum](https://esp32.com/viewtopic.php?t=6393)
- [joltwallet/littlefs, ESP Component Registry](https://components.espressif.com/components/joltwallet/littlefs)
- [esp_littlefs, joltwallet](https://github.com/joltwallet/esp_littlefs)
- [Storage Media, ESP-IoT-Solution](https://espressif-docs.readthedocs-hosted.com/projects/espressif-esp-iot-solution/en/latest/storage/storage_media.html)
