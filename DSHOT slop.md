# DShot


Design for the ESC signal path. Replaces the `motor` component with two
components, `dshot` and `esc`, plus wheel-level concerns moving into `vehicle`.


## Terms

Only the words that appear in code.

| Term | Meaning |
|---|---|
| ESC | the device on the far end of the signal wire. Arms, holds settings, answers telemetry |
| link | one signal wire and the RMT channel pair bound to it |
| frame | one 16-bit DShot transmission |
| reply | the ESC's 21-bit GCR burst following a frame |
| direction | requested travel direction, changes at runtime |
| mounting | fixed physical orientation of a motor, set once per build |
| wheel | a vehicle corner |

`gear` is not used. `inverted` is not used for mounting.

## Layers

| Component | Owns | Does not know about |
|---|---|---|
| `dshot` | RMT channels, GPIO, frame generation, symbol building, CRC, GCR decode, EDT demux, turnaround | AM32, arming, car mode, direction, mounting, logging |
| `esc` | arming, EDT enable, command sequences, AM32 car-mode value mapping, direction changes, watchdog, telemetry cache | RMT, GPIO, timers, vehicle geometry |
| `vehicle` | corner identity, mounting, differential, torque split, publishing | DShot values, ESC state machine internals |

`dshot` depends on `driver`, `hal` and `esp_timer` and nothing else in this
repo, so it can be published on its own. `esc` depends on `dshot` only. Neither
knows about the blackboard, which removes the `Topic<motor::RpmReading>`
specialization currently sitting in `motor.h`.

## dshot::Link

One object per signal wire. It acquires its own RMT TX channel, its own RX
channel when bidirectional, and fails construction when none are free. There is
no container object and no registry. The RMT bus clock and the shared interrupt
handler are refcounted inside the component, allocated on the first `Link` and
released with the last.

```cpp
namespace dshot {

enum class Protocol : uint8_t { D150, D300, D600, D1200 };
enum class Pad      : uint8_t { PushPull, OpenDrain };

struct Config {
    gpio_num_t gpio;
    Protocol   protocol{Protocol::D600};
    uint32_t   frame_hz{2000};
    bool       bidirectional{true};
    Pad        pad{Pad::PushPull};
};

enum class Freshness : uint8_t { NoData, Stopped, Valid };

struct Erpm { uint32_t value; Freshness state; };

struct Telemetry {
    uint32_t erpm;     uint32_t erpm_us;
    uint8_t  temp_c;   uint32_t temp_us;
    uint16_t volt_cv;  uint32_t volt_us;
    uint8_t  amps;     uint32_t amps_us;
    uint8_t  stress;   uint32_t stress_us;
    uint16_t status;   uint32_t status_us;
};

struct Health {
    uint32_t frames;
    uint32_t replies;
    uint32_t crc_fail;
    uint32_t no_reply;
    uint32_t torn_write;
};

class Link {
public:
    explicit Link(const Config&);
    ~Link();
    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;

    esp_err_t start();
    void      stop();

    void     set(uint16_t value, bool command_bit = false);
    uint16_t value() const;

    Erpm      erpm() const;
    Telemetry telemetry() const;
    Health    health() const;
};

}
```

`set()` is the entire write API. `value` is the 11-bit payload, 0 to 2047. The
component does not interpret it. 1048 means nothing here.

Every field in `Telemetry` carries its own timestamp. With EDT enabled roughly
three percent of replies are not eRPM, and a shared timestamp would make a
current frame read as a missed eRPM reading.

`Freshness` separates three cases that a plain zero collapses. `Valid` is a
decoded reading, `Stopped` is AM32's 0x0FFF standstill code, `NoData` is a CRC
failure or silence. The standstill check needs `Stopped` to mean something
different from a silent ESC.

## Frame generation

Frames come from RMT continuous mode. No timer, no task.

Sixteen symbols hold the frame, one more holds the gap to the next frame. A
single RMT symbol encodes up to 819 microseconds at 40 MHz, so any frame rate
above roughly 1.2 kHz fits in one gap symbol. Seventeen words out of the 48 each
channel owns. Setting `tx_conti_mode` makes the channel replay that memory
forever with no CPU involvement, and the frame rate is whatever sits in the gap
symbol. Rate is per channel, so nothing is shared between links.

Changing throttle means rewriting the sixteen frame symbols in place. Two
properties bound the risk.

A torn frame fails CRC. The CRC occupies the last four symbols, so any mix of
old and new produces a mismatch and the ESC drops the frame instead of acting on
a wrong value.

The transmitter's read offset is readable in `mem_raddr_ex_chn`. Check it is
past symbol 16 before writing and check again after, and the update is verified
rather than assumed. A crossing increments `torn_write` and the value is
rewritten on the next period.

Two interrupts per frame per link, both on the single shared RMT interrupt line,
so a handler entry services every channel that fired.

| Event | Work |
|---|---|
| TX threshold, fires after symbol 16 | switch pad to open-drain when in push-pull mode, reset and enable the receiver |
| RX done | read symbols, GCR decode, EDT demux, store with timestamp, disable receiver |

Frame timing is generated by hardware, so period jitter is zero.

### Why not the ESP-IDF driver

`rmt_transmit()` gives no way to touch channel memory while a channel runs, so
continuous mode under IDF requires `rmt_disable()` and `rmt_enable()` for every
value change. The frame path is written against `hal/rmt_ll.h` and the channel
memory directly. See [Rejected alternatives](#rejected-alternatives) for the
cost of the other route.

The component assumes exclusive use of the RMT peripheral. There is no way to
ask the IDF driver what it has claimed. Nothing else in this repo uses RMT, as
`rgb_led` runs on SPI3.

The interrupt handler and the update path are IRAM resident, so neither logs.
Counters carry the diagnostics.

## Telemetry decode

The reply is 21 GCR bits at 5/4 the DShot bit rate, one per frame. DShot600
sends at 600 kbit/s and the reply comes back at 750 kbit/s, which puts the
reply at 28 microseconds against the frame's 26.7.

After GCR decode and CRC the payload is 12 bits.

| Condition | Meaning |
|---|---|
| `value == 0x0FFF` | motor stopped |
| `(value & 0x100) == 0 && (value & 0xE00) != 0` | EDT frame, type `(value >> 8) & 0x0F`, data `value & 0xFF` |
| otherwise | eRPM, period `(value & 0x1FF) << (value >> 9)` |

The EDT space is reachable because eRPM values are normalized. With a nonzero
exponent the mantissa's top bit is always set, so a nonzero exponent together
with a clear bit 8 cannot come from an eRPM reading.

| Type nibble | Field |
|---|---|
| 0x2 | temperature, degrees C |
| 0x4 | voltage, 0.25 V steps |
| 0x6 | current, 1 A steps |
| 0x8, 0xA | debug |
| 0xC | stress level |
| 0xE | status and version |

AM32 sends current every 40 frames and temperature and voltage every 200.

## esc::Esc

One per link. The `esc` component owns the AM32 semantics and nothing else.

```cpp
namespace esc {

enum class Mounting  : uint8_t { Normal, Mirrored };
enum class Direction : uint8_t { Forward, Reverse };
enum class State     : uint8_t { Stopped, Arming, Configuring, Reversing, Ready };
enum class Beep      : uint8_t { One = 1, Two, Three, Four, Five };

struct Config {
    dshot::Config link;
    Mounting mounting{Mounting::Normal};
    uint8_t  poles{14};
    bool     edt{true};
    uint16_t arm_hold_ms{1500};
    uint16_t command_timeout_ms{250};
};

struct Rpm { int32_t value; dshot::Freshness state; };

class Esc {
public:
    explicit Esc(const Config&);

    esp_err_t start();
    void      tick();

    void arm();
    void disarm();

    void command(int16_t torque);   // -999 brake .. +999 drive
    void drive(float t);            // 0..1
    void brake(float t);            // 0..1
    void coast();

    bool      direction(Direction);
    Direction direction() const;
    bool      beep(Beep);

    State            state() const;
    Rpm              rpm() const;
    dshot::Telemetry telemetry() const;
    dshot::Health    health() const;
};

}
```

`tick()` is called from the application's control loop at 200 Hz or above. There
is no task in this component. The sequences need millisecond resolution and hold
values for tens of milliseconds, so a 500 Hz control loop has ample margin.

`drive()` and `brake()` are the same actuator. AM32 cannot blend them, so both
map onto one signed value and `command()` is the primitive.

### Value mapping

AM32 with `rc_car_reverse = 1` splits the 11-bit range into two 1000-step
halves. Each half's base value is that half's zero point, because AM32 computes
`((value - base) * 2 + 47) - reversing_dead_band` and drives only at 47 or
above.

`Mounting` picks which half is that motor's forward. `Direction` picks which
half is currently the drive half. The other half is the brake half.

| `command()` | Sent | ESC behaviour |
|---|---|---|
| `+1 .. +999` | drive base + n | drives |
| `0` | drive base | freewheels, both FETs off with a phase floating |
| `-1 .. -977` | brake base + n | proportional brake, duty `n * 2 + 46` out of 2000 |
| `-978 .. -999` | brake base + n | full brake, hard short |

Coast is the drive half's base, not DShot 0. DShot 0 arms AM32's
`return_to_center` and is the only value that can flip direction, so it is
emitted only by the sequences below.

Low half base is 48, high half base is 1048, span is 999 either way.

### Sequences

Four things need a value held for a duration. Each is a value, a hold in
milliseconds, and the next step. Durations are milliseconds and never frame
counts, because a frame count silently changes meaning when the rate changes.

**Arming.** Hold DShot 0. AM32 needs about a second of zero throttle after it
has locked onto the signal, and the lock takes a variable few seconds. Telemetry
returning is the evidence of lock, so the hold is timed from the first reply
with a timeout for a link that never answers.

**Configuring.** DShot command 13 at least six times to enable EDT. The ESC
answers with a version frame. Skipped when `edt` is false.

**Reversing.** AM32 will not switch from braking to driving the other way until
a value below 48 arrives while it is braking. Command the target half, drop
below 48, command the target half again. A motor already facing the target way
passes through with no effect, so the same sequence serves the post-arm
establish and a later change. Only safe at standstill, and `Esc` does not check
that. The vehicle controller owns the decision.

**Beeping.** Command 1 through 5 held long enough for six frames. AM32 only
processes commands with the motor stopped, so this coasts first.

### Watchdog

No call to `command()`, `drive()` or `brake()` within `command_timeout_ms`
coasts the motor. Coast, not brake.

A stopped `tick()` leaves the ESCs receiving the last value indefinitely, since
frame generation is in hardware. The vehicle layer is responsible for noticing
that, using `Health::replies` rather than inferring it from zero RPM.

## Rate

### Wire time, bidirectional

Turnaround is the ESC's and the figure below is the BLHeli and AM32 convention.

| Protocol | Frame | Turnaround | Reply | Idle detect | Total | Ceiling | With 25% guard |
|---|---|---|---|---|---|---|---|
| DShot150 | 107 us | 30 | 112 | 30 | 279 | 3.6 kHz | 2.8 kHz |
| DShot300 | 53 | 30 | 56 | 15 | 154 | 6.5 kHz | 5.2 kHz |
| DShot600 | 27 | 30 | 28 | 8 | 93 | 10.7 kHz | 8.6 kHz |
| DShot1200 | 13 | 30 | 14 | 4 | 61 | 16 kHz | 13 kHz |

8 kHz bidirectional requires DShot600 or above. DShot300 cannot round-trip
inside 125 microseconds. `Config` validation computes this from the protocol
constants and refuses an impossible pairing at construction.

RMT resolution is 40 MHz through DShot600. DShot1200 needs 80 MHz, because at
40 MHz a bit is 33 ticks and the quantisation error reaches three percent of the
bit.

### CPU cost

Per frame, four links, 240 MHz. Derived from instruction counts and bus timing
rather than measured.

| | us per frame |
|---|---|
| TX threshold interrupt entry | 1.5 |
| pad switch, 4 links | 0.1 |
| arm receiver, 4 links | 0.5 |
| RX done interrupt entry | 1.5 |
| GCR decode, 4 links | 3.6 |
| **total** | **7.2** |

| Frame rate | Share of one core |
|---|---|
| 2 kHz | 1.4% |
| 8 kHz | 5.8% |

Symbol rebuilds happen at the control rate, not the frame rate, and cost about
1.1 microseconds per link that changed value. At 500 Hz across four links that
is under a quarter of one percent.

Cost is protocol independent. Building sixteen symbols and decoding 21 GCR bits
is the same work at DShot150 as at DShot1200.

## Peripheral budget

Maximum bidirectional links is `min(TX channels, RX channels)`.

| Chip | RMT channels | Max bidirectional links |
|---|---|---|
| ESP32 | 8, either role | 4 |
| ESP32-S2 | 4, either role | 2 |
| ESP32-S3 | 4 TX + 4 RX, fixed roles | 4 |
| ESP32-C3 | 2 TX + 2 RX | 2 |
| ESP32-C6 | 2 TX + 2 RX | 2 |
| ESP32-H2 | 2 TX + 2 RX | 2 |

Four links on a C3 is not possible over RMT with or without telemetry, because
it has two TX-capable channels. The limit comes from
`SOC_RMT_TX_CANDIDATES_PER_GROUP` and constructing one link past it returns an
error. No per-chip code.

Continuous mode uses no general purpose timers. The S3 has four in total and the
C3 has two, so a design with one timer per link would consume every timer on the
S3 and be impossible on the C3 at four links.

## Wiring

Bidirectional DShot puts two push-pull outputs on one wire. Both driving at once
in opposite directions leaves two transistors of a few tens of ohms in series
between 3.3 V and ground.

`Pad::PushPull` switches the pad to open-drain for the receive window and back
for transmit. `Pad::OpenDrain` leaves it open-drain permanently and an external
pull-up makes every rising edge, which removes the contention case entirely.

Open-drain applies to bidirectional links only. Regular DShot idles low and its
bits are high pulses, so a pull-up would have to generate every data pulse. At
DShot600 the shortest high period is 419 ns and a 1k pull-up against roughly
100 pF crosses the input threshold in about 110 ns. It works and gets worse
above 600. Regular DShot has one device on the wire and wants push-pull.

One board layout supports both modes. Fit a 1k pull-up per signal line and a
100 ohm series resistor.

| | Open-drain | Push-pull |
|---|---|---|
| 1k pull-up | required for rise time | wastes 3.3 mA per line while driving low |
| 100 ohm series | low level sits at 0.30 V | limits a contention event to about 18 mA |

A 330 ohm series resistor puts the open-drain low level at 0.82 V, which is too
close to the input threshold. That is why the series value drops when a pull-up
is present.

DShot1200 in open-drain wants 470 ohms instead of 1k, doubling the wasted
current and halving the rise time.

The pull-up also gives the line a defined idle level before the firmware
configures the pin, which is the correct state for a bidirectional ESC and
harmless for a regular one.

## Rejected alternatives

### gptimer plus rmt_transmit()

Works and is much simpler to write. The cost is CPU.

| | us per frame, 4 links |
|---|---|
| gptimer alarm interrupt | 2.0 |
| notify and context switch | 2.5 |
| build symbols | 2.0 |
| `rmt_transmit()` x4 | 18.0 |
| `rmt_receive()` x4 | 11.2 |
| IDF per-channel interrupt dispatch x8 | 8.0 |
| interrupt entry, 2 to 8 entries | 3.0 to 12.0 |
| GCR decode | 3.6 |
| **total** | **50 to 58** |

That is 10 to 12 percent of one core at 2 kHz and 40 to 46 percent at 8 kHz. The
clock accounts for 4.5 microseconds of it. The remaining 37 are the driver
calls, so keeping the timer and writing registers directly lands at 16.2
microseconds.

Beyond cycles it brings a transmit queue, and therefore a queue-full path that
drops frames silently, a task at priority 20 competing with the IMU and RC link,
and frame jitter equal to task scheduling jitter.

### LCD_CAM parallel bitbang

The only route past four bidirectional links on an S3, and the way to get eight.

The LCD half drives 8 or 16 parallel output lines from a GDMA stream on a
free-running clock, and a looping descriptor makes transmit as cheap as RMT
continuous mode. The CAM half samples the same lines back through the GPIO
matrix, so eight pads serve eight bidirectional links.

Receive is where it loses. RMT's receiver is a hardware edge timer and hands
back level and duration pairs, so a 21-bit reply arrives as about eleven
entries. CAM hands back raw samples, so the same reply is around 126 samples per
link and the edges are found in software. Ten times the data per link, and that
decode is the dominant cost of bitbang DShot on flight controllers. For eight
links it lands around 11 to 14 microseconds per frame depending on sampling rate
and whether capture can be gated to the reply window, which needs the CAM VSYNC
input driven from somewhere and probably costs a pin.

Betaflight bitbangs because STM32 has only timers and DMA. Discarding a
purpose-built edge-timing peripheral to gain a link count that is not needed is
a poor trade at four links.

## To confirm during implementation

Whether the end marker wraps or halts the channel with `tx_conti_mode` set, and
whether the TX threshold interrupt re-fires on every loop iteration. Both are in
the S3 TRM RMT chapter.
