# Blackboard

Design for the system-wide data bus that replaces the current `telemetry` component.
Status: design, not yet implemented.

## Why

Two problems with the current setup:

1. **Split-brain data access.** Some signals flow through the `telemetry`
   dispatcher (RPM, GPS), others are pulled directly from producer singletons
   (`imu::IMU::instance()`, `PowerMonitor::instance()`, `rclink::Receiver::instance()`).
   The vehicle controller reaches straight into three components, so it depends
   on all of them.

2. **No single place to log everything.** The goal is to capture every signal
   the vehicle produces and use it offline to build behavioural models. That is
   only practical if all data passes through one chokepoint.

The existing `telemetry` component is already a multi-producer multi-consumer
pub/sub bus. This design finishes that idea, fixes a correctness bug in it, and
makes it the single point every signal flows through.

## Naming

The component is renamed `telemetry` to `blackboard`. "Blackboard" is the
established name for this pattern: a shared store of latest values that many
producers write and many consumers read.

`telemetry` was a misleading name because telemetry already means something
specific in this codebase, the RC downlink to the transmitter
(`rclink::send_battery / send_gps / ...`). That code keeps the name telemetry
because that is genuinely what it is. The downlink becomes one consumer of the
blackboard among several.

## Type ownership

The blackboard does **not** define the data types it stores. Each producing
component owns its own type and the blackboard is generic over it. This is
deliberate and is the central rule of the design:

- The IMU already owns `imu::AccelData / GyroData / QuatData / EulerData`. The
  RC layer owns `rclink::RcChannels`. The blackboard stores those exact types,
  not copies.
- If a producer changes its data, it changes one struct in one component. The
  blackboard, being generic, needs no edit.
- The stored type is **not** limited to what EdgeTX/CRSF telemetry can carry.

The CRSF-shaped structs currently in `telemetry_types.h` (`BatteryTelemetry`,
`AccelGyroTelemetry`, ...) are a **wire format for the downlink**, not a storage
format. They move to live with the RC downlink consumer. The downlink reads the
rich producer type from the blackboard with `latest<T>()` and converts it down
to the constrained CRSF representation on the way out. Conversion is the
consumer's job, not the producer's and not the bus's.

Consequence the blackboard core depends on **no producer component**. Producers
depend on the blackboard. The only thing that must be centralised is the mapping
from type to `topic_id`, covered below, and that is done without the blackboard
naming any concrete type.

One gap today: `PowerMonitor` exposes scalar getters
(`get_voltage_mv()` and friends) and has no struct. It needs a small interface
type, for example `power_monitor::Reading { int32_t voltage_mv; int32_t
current_ma; uint32_t consumed_mah; uint8_t remaining_pct; }`, which becomes the
type it publishes. The downlink converts that to `BatteryTelemetry` for CRSF.

## The core constraint

Every topic that has a live consumer today gets a second consumer (the logger)
once logging exists. So **every read path must be non-consuming**. A design
where reading a value removes it (the current `xQueueReceive` on a depth-1
mailbox) breaks the moment two consumers race to empty the same mailbox. This is
a latent bug in the current `telemetry` component.

## Two planes

`publish()` feeds two independent mechanisms at the same time. They serve
different masters and must not be conflated.

```
                        publish(sample)
                              |
              +---------------+----------------+
              |                                |
        seqlock snapshot                  global log ring
        (one slot per topic)              (one ring, all topics)
              |                                |
        live consumers                     logger task
        copy, never consume                drains to a sink
        (control loop, downlink)           (TCP / SD / stdout)
```

### Plane 1: latest-value snapshot (live reads)

One slot per topic holding the most recent sample. Consumers **copy** it, never
consume it. Unlimited readers, no racing, nobody steals a sample from anyone.
The control loop, the ELRS downlink, and any future consumer read here.

Do **not** use `std::atomic<T>` for the struct types. It is only lock-free for
small types; for anything the size of `RcChannels` the standard library falls
back to an internal mutex, which is a hidden lock on the 500 Hz control path.

Use a **seqlock**:

```cpp
template <typename T>
class Snapshot {
    std::atomic<uint32_t> seq_{0};
    T value_{};

public:
    void store(const T& v) {                  // single writer (the producer)
        seq_.fetch_add(1, std::memory_order_acquire);   // -> odd: write in progress
        value_ = v;
        seq_.fetch_add(1, std::memory_order_release);   // -> even: done
    }

    T load() const {                          // many readers, never blocks
        T out;
        uint32_t s0, s1;
        do {
            s0 = seq_.load(std::memory_order_acquire);
            out = value_;
            s1 = seq_.load(std::memory_order_relaxed);
        } while ((s0 & 1) || s0 != s1);       // retry if writer touched it mid-read
        return out;
    }
};
```

Single writer, many readers, never blocks, always coherent. A few lines and no
OS primitive. One writer per topic is assumed; the producer of a topic is its
only writer.

### Plane 2: global tagged log ring (history)

One ring for the **whole system**, not per-topic and not per-subscriber. Every
`publish()` appends a record `[topic_id][timestamp_us][payload]` to a single
multi-producer ring. The logger drains that one ring sequentially to its sink.

One global ring rather than per-topic rings because:

- **Time ordering for free.** Records land interleaved in real time, which is
  exactly what is needed to model cause and effect across signals. No merging N
  files by timestamp offline.
- **Adding a topic is a new `topic_id` and nothing else.** The logger never
  names a type.

Use ESP-IDF `RingbufHandle_t` with `RINGBUF_TYPE_NOSPLIT`:

- multi-producer safe (internal lock only on the brief byte copy at send time,
  not on the seqlock read path),
- variable-size items, which suits heterogeneous records,
- single consumer (the logger).

If the ring is full, `publish()` drops the record (non-blocking) and bumps a
dropped-record counter. Producers, especially the control loop, must never block
on logging.

## Record format

```
offset  size  field
0       1     topic_id        (see table)
1       1     payload_len     (bytes)
2       6     timestamp_us    (48-bit microseconds since boot, little-endian)
8       N     payload         (raw struct bytes for that topic)
```

48-bit microseconds wraps after ~8900 years, so no rollover handling. The
timestamp is stamped at `publish()` time inside the blackboard, so every record
shares one clock regardless of which task produced it. The payload is the raw
producer struct.

Because the payload is the producer's own struct and those are free to evolve,
the offline decoder must know each `topic_id`'s layout for the firmware version
that produced the log. Keep a schema reference (this document plus the struct
definitions) versioned alongside the firmware. If logs need to stay decodable
across struct changes, add a one-byte `schema_version` per topic to the record
header; left out of the first cut to keep records small.

## Topic IDs

The type for each topic is owned by its producing component. IDs are stable and
append-only; never renumber, only add. The id numbers live in one
dependency-free enum so collisions are impossible to miss (see the registry
section); the types stay in their components.

| topic_id | Topic        | Type (owning component)       | Producer       |
|----------|--------------|-------------------------------|----------------|
| 0        | reserved     | -                             | -              |
| 1        | battery      | `power_monitor::Reading` (new)| power_monitor  |
| 2        | gps          | gps fix type (gps/nmea)       | nmea driver    |
| 3        | euler        | `imu::EulerData`              | imu            |
| 4        | link_stats   | rc link-stats type (rc)       | rc backend     |
| 5        | rc_channels  | `rclink::RcChannels`          | rc backend     |
| 6        | rpm          | vehicle rpm type (vehicle)    | vehicle        |
| 7        | flight_mode  | vehicle mode type (vehicle)   | vehicle        |
| 8        | accel        | `imu::AccelData`              | imu            |
| 9        | gyro         | `imu::GyroData`               | imu            |
| 10       | quat         | `imu::QuatData`               | imu            |

`rc_channels` is published instead of pulled from the `rclink::Receiver`
singleton, so the controller and the logger both read it from the blackboard.
`accel`, `gyro`, `quat`, `euler` are stored as the IMU's own types rather than
flattened into one CRSF `AccelGyroTelemetry`; the downlink recombines whatever it
needs for the wire.

## Generic channels and type-to-id binding

The current component hand-writes a `publish` overload, a `poll_latest`
overload, a static channel, and a logger drain block per type. That per-type
repetition is what fights the "log everything from one place" goal.

The blackboard core is generic over `T` and names no concrete type. Three
pieces, in dependency order:

**1. A dependency-free id enum** (`blackboard/topic_ids.h`). Just numbers, no
type includes. This is the one central place that prevents id collisions and the
reference the offline decoder reads.

```cpp
namespace blackboard {
enum class TopicId : uint8_t {
    reserved = 0,
    battery = 1, gps = 2, euler = 3, link_stats = 4, rc_channels = 5,
    rpm = 6, flight_mode = 7, accel = 8, gyro = 9, quat = 10,
    // append only, never renumber
};
}
```

**2. The generic channel and a `Topic<T>` trait** (blackboard core). The trait
primary template is declared but not defined, so publishing a type that has not
been bound to an id is a compile error rather than a silent default.

```cpp
namespace blackboard {

template <typename T> struct Topic;   // specialise per type to bind T -> id

template <typename T>
class Channel {
    Snapshot<T> snap_;
public:
    void publish(const T& v) {
        snap_.store(v);
        ring_append(static_cast<uint8_t>(Topic<T>::id), &v, sizeof(T));
    }
    T latest() const { return snap_.load(); }
};

template <typename T>
Channel<T>& channel_for() {            // one instance per type, lazy, thread-safe
    static Channel<T> ch;
    return ch;
}

template <typename T> void publish(const T& v) { channel_for<T>().publish(v); }
template <typename T> T    latest()            { return channel_for<T>().latest(); }
}
```

`ring_append(topic_id, bytes, len)` is a plain non-template function in
`blackboard.cpp`. It is the only thing that touches the global ring, and it works
in raw bytes, so it never names a concrete type.

**3. One trait specialisation per type, in the owning component's header**, next
to the type it binds. The component already includes its own type header and the
tiny `topic_ids.h`, so this adds no cross-component dependency. Producer A never
pulls in producer B's headers.

```cpp
// in imu_types.h, beside the struct definitions
#include "blackboard/topic_ids.h"
namespace blackboard {
template <> struct Topic<imu::EulerData> { static constexpr TopicId id = TopicId::euler; };
template <> struct Topic<imu::AccelData> { static constexpr TopicId id = TopicId::accel; };
template <> struct Topic<imu::GyroData>  { static constexpr TopicId id = TopicId::gyro;  };
template <> struct Topic<imu::QuatData>  { static constexpr TopicId id = TopicId::quat;  };
}
```

Adding a topic is: define the struct in its component, add one enum value to
`topic_ids.h`, add one trait specialisation beside the struct. The blackboard
core, the ring, and the logger are untouched because they only ever see
`topic_id` and raw bytes.

No central `init()` and no channel list are needed; `channel_for<T>()`'s
function-local static instantiates each channel on first use. The logger does not
enumerate channels either, it drains the single global ring.

## Logger

A separate component, `logger`, not part of the blackboard. The blackboard owns
the buffer; the logger owns the policy. This split is the whole point of the
earlier discussion: the buffer must live at the publish chokepoint to capture
every sample, but when and where to flush is not the blackboard's concern.

The buffer lives in the blackboard rather than the logger so producers never
have to know the logger exists. If the logger held its own buffer, producers
would have to publish to it directly, which is the coupling this design removes.

```cpp
class Sink {
public:
    virtual ~Sink() = default;
    virtual bool write(const uint8_t* data, size_t len) = 0;  // false if it could not take it
};
```

The logger task loop:

1. `xRingbufferReceive` from the global ring (block with a short timeout).
2. Hand the bytes to the active `Sink`.
3. Repeat.

Swapping sinks (stdout for bring-up, TCP for real capture, SD later) does not
touch the blackboard, the channels, or the producers.

## Sinks

### StdoutSink (bring-up)

Writes records to the console. Zero hardware. Bandwidth-limited (~11 KB/s at
115200 baud) so it cannot keep up with full-rate IMU, but it proves the ring
drains and lets you eyeball records. The default sink during development.

### TcpSink with PSRAM store-and-forward (real capture)

The board is an N16R8: 16 MB flash, 8 MB octal PSRAM. The plan is to stream
records to a laptop over TCP, using a large PSRAM ring as a store-and-forward
backlog for when WiFi is out of range.

```
ring (SRAM, MPSC)  ->  logger  ->  PSRAM backlog ring  ->  TCP socket  ->  laptop
```

- When the socket can send: drain the PSRAM backlog first, then send live
  records, so the stream stays in time order.
- When the socket is down (out of range, laptop closed): accumulate into the
  PSRAM backlog.
- The logger does not care why the socket is down; it just sees "cannot send
  now."

Backlog sizing: at ~25-30 KB/s full rate, 8 MB buys roughly 4-5 minutes of
backlog before the ring wraps. Wrap policy is **overwrite oldest**: on an outage
longer than the backlog, keep the most recent few minutes and lose the middle of
the gap. Acceptable for modelling, but a conscious choice.

PSRAM notes:

- PSRAM is volatile. It is a buffer, never a savefile. Anything that must
  survive a power cycle goes to flash or SD.
- Currently disabled (`CONFIG_SPIRAM` is not set). Enabling needs octal mode at
  80 MHz in menuconfig for the R8 part.
- Allocate the backlog with `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)` so it is
  pinned to PSRAM and never eats internal SRAM.
- PSRAM and WiFi share the SPI/cache subsystem. Harmless at tens of KB/s, worth
  remembering if rates climb much later.

### SdCardSink (later, untethered runs)

For sessions with no laptop attached. FATFS is already in the build and the S3
supports SDMMC. Needs wiring and a mode choice (SDMMC 1/4-bit vs SPI) against
the free GPIOs. A PSRAM staging buffer in front of it absorbs the periodic
tens-of-millisecond write stalls SD cards exhibit, so producers never back up.
Not needed for the first cut.

## Migration path

The blackboard can be stood up alongside the existing code and producers moved
over one at a time.

1. **Done.** Built `blackboard` (seqlock channel, global ring, `topic_ids.h` +
   `Topic<T>` trait) and `logger` with `StdoutSink`. Records drain on hardware.
2. **Done.** Producers publish their own interface types (additive, the old
   `telemetry` calls still run alongside):
   - `PowerMonitor::Reading` (new) -> battery.
   - imu publishes `AccelData`/`GyroData`/`QuatData`/`EulerData`.
   - rc backend publishes `rclink::RcChannels` once per frame in ELRS `run()`.
   - nav adds `nav::GpsFix` (new), published by the NMEA driver.
   - vehicle adds `vehicle::MotorRpm` (new), published in `control_motors`.
   - `link_stats` and `flight_mode` are skipped: no producer for the former, the
     latter is only a GPS debug string today.
3. **Done.** Consumers read `latest<T>()`; the old `telemetry` component is
   deleted entirely.
   - The ELRS downlink (`ExpressLRS::send_telemetry`) reads the producer types
     from the blackboard and converts them to CRSF wire frames inline. The
     former `telemetry_types.h` structs are gone; conversion lives in the
     downlink, not in producers.
   - The vehicle controller reads `gyro`/`euler` and the power reading from the
     blackboard, dropping its dependency on the imu and power_monitor singletons.
   - RC stays on `rclink::Receiver` for the control loop: link validity
     (`valid_data()`) is a safety signal not carried on the bus, and the
     `Receiver` atomic snapshot is the right read mechanism. `rc_channels` is
     still published to the blackboard for logging, so the two paths are
     control (Receiver) and recording (bus), not duplicate state.
4. **Done.** Removed the now-dead getters and backing storage: IMU lost its
   `get_*`/`has_valid_*` methods, mutex, and cached samples (it is now a pure
   producer that publishes in `push_*_from_backend`); PowerMonitor lost its
   scalar getters and atomics (it publishes a `Reading`, which gained
   `cell_count`). The rpm bus type lives in `motor` (`motor::RpmReading`) to
   avoid a vehicle↔rc include cycle.
5. **In progress.** `TcpSink` (`components/tcp_sink`) is done: SoftAP +
   single-client TCP server + 4 MB PSRAM overwrite-oldest store-and-forward ring.
   Read it with `tools/telemetry_listener.py`. `SdCardSink` for untethered runs
   is still to do.

## What stays off the bus

Commands, meaning the controller driving the motors and servos, stay direct.
That path must be deterministic and immediate, so it is not routed through a
pub/sub queue. The blackboard is for data flowing out of producers, not for
actuation flowing into actuators.
