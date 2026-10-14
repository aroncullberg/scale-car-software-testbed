# DShot

This is a document outlining the design of the new esc signal path and driver. It replaces the `motor` component with two components, `dshot` and `esc`.

## What this needs to do

Four bidirectional links on the ESP32-S3, each sending at least 4000 frames/s. They run in parallel, so each wire gets 250 us at 4 kHz. DShot600 is the starting point, with 8 kHz as the next rate to try. DShot150/300 should still be available where the requested frame rate allows them. DShot1200 is possible with the S3 clock, but support on our ESC still needs checking.

I want to be able to switch between regular and bidirectional DShot in software without changing the wiring. So the first implementation should use push-pull while transmitting and disable the output while receiving, with the internal pull-up holding idle high. No required external pull-up. The timing of that handover needs testing on all four wires.

The current proposal is one shared GPTimer starting four RMT transmissions, with direct register and memory access in the driver. The continuous-loop idea is kept below as an alternative. TODO: measure both before calling one cheaper.

## Components

| Component/namespace | Responsible for |
|---|---|
| `dshot` | RMT channel setup, frame generation, CRC, GCR decode, EDT demux, half-duplex turnaround |
| `esc` | arming, EDT enable, mode-dependent value mapping, direction changes, command watchdog policy, mechanical RPM |
| `vehicle` | corner identity, mounting, differential, torque split, publishing |

One goal is to have `dshot` not depend on any code from the rest of the repo, to allow it to be published as a standalone library.

## dshot::Link

One `Link` per signal wire. Each claims its own RMT TX channel, and an RX channel when bidirectional, from a bitmap of free channels under a spinlock. Claiming is a scan for a free index rather than a sequence of attempts, so a full chip produces one `ESP_ERR_NOT_FOUND` and no noise.

```cpp
namespace dshot {

enum class Protocol : uint8_t { D150, D300, D600, D1200 };

// Use the same settings for all active links. This is our choice to keep scheduling simple.
struct Wire {
    Protocol protocol{Protocol::D600};
    uint32_t frame_hz{4000};
    bool     bidirectional{true};
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
    uint32_t late_frames;
    uint32_t rx_overrun;
    uint32_t watchdog_trips;
};

class Link {
public:
    Link(const Wire&, gpio_num_t);
    ~Link();
    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;

    esp_err_t begin();
    void      end();

    void      set(uint16_t value, bool telemetry_bit = false);
    uint16_t value() const;

    Erpm      erpm() const;
    Telemetry telemetry() const;
    Health    health() const;
};

}
```

The only public facing write api is `set()`, `value` is the 11-bit payload. So its 0-2047. Its supposed to be 'dumb' so `dshot` does not interpret it, simply sends it along to the esc. That includes commands 0-47, with their repeats and delays controlled from the `esc` layer. The extra bit is the protocol's telemetry-request bit, not a separate command flag.

`set()` publishes a pending value in RAM. The driver takes it at the next frame boundary, so calling it does not write into a running RMT channel. `value()` returns the latest requested value. TODO: expose when a value actually went onto the wire and how many complete frames used it, so `esc` can time command sequences from that point. Also decide the small API needed for a fallback value and command expiry. Those are still sketches.

<!--i rewrote the next section as the section above intead, was too sloppy for me. but it did give me the thought of things such as commands which needs more delay or repeats how should we do with that?-->
<!--`set()` is the entire write API. `value` is the 11-bit payload, 0 to 2047, and
`dshot` does not interpret it. The split into halves, what counts as coast, and
which values are commands are all `esc` concerns.-->

Important to note is that the `dshot::Link` holds a channel "claim" so it cannot be copied or moved. The two copies would both think they own the same RMT channel, and the shared driver keeps a pointer to the object so moving it would leave the ISR pointing at the old one.

<!--again the next part was too sloppy for my taste, idk try to identify what is different in these two texts so we can generalize it and prevent it in the future.-->
<!--i dont see the reason to state the second half of the following tbh, but please do try and convince me-->
<!--A `Link` holds an interrupt handle and a channel claim, so it is neither copyable nor movable. The same follows for any `esc::Esc` that owns one. That suits declaring them as statics and rules out putting them in a `std::vector`.-->

### Shared peripheral state

There is a small shared object inside `dshot` holding the channel bitmap, clock setup, interrupt handle and timer. The first `begin()` initializes it and the last `end()` releases it. Starting a second link must not reset the first one. This does not need to be another object the application has to create.

The S3 has a shared RMT clock source and group divider, then another divider per TX/RX channel. So identical resolutions are a choice here, not a hardware requirement. Start with 40 MHz from XTAL for DShot150/300/600. If we add DShot1200, use 80 MHz from APB. Power management is off in this project; a standalone build using APB needs to keep that clock stable and prevent sleep while the links run.

All RMT channels notify the CPU through the same interrupt source. The status bits say which channel and event need handling. This has nothing to do with connecting the signal wires together. The proposed driver uses one handler to service the active links, releasing any completed TX pads before doing telemetry decoding. IDF uses per-channel handlers with status masks, which is also possible.

### Exclusive use of the RMT peripheral

This implementation owns RMT while any link is active. Do not allocate other channels through the IDF RMT driver at the same time, even if some channels are free.

The old idea of checking a peripheral refcount does not reliably detect that conflict. In IDF 5.5.1, RMT keeps its own private group count and can reset the peripheral when its first channel is created. Our bitmap cannot see its allocations. So exclusive use is a requirement for now, and automatic conflict detection is still a TODO.

## esc::Esc

`Esc` is a holding ojbect for the link to make it easier to interact with and to better delimit responsibilities, as such one `Esc` owns one link. It deals with the translation between intent and dshot/am32 logic/semantics.
<!--One `Esc` per link, owning that link by value. `Esc` holds the AM32 semantics and nothing else.-->
<!--I take great issue with how you parrot the ...and nothing else phrase-->

```cpp
namespace esc {

enum class Mode      : uint8_t { Default, ThreeD, Car };
enum class Mounting  : uint8_t { Normal, Mirrored };
enum class Direction : uint8_t { Forward, Reverse };
enum class State     : uint8_t { Stopped, Arming, Configuring, Reversing, Ready };
enum class Beep      : uint8_t { One = 1, Two, Three, Four, Five };

struct Config {
    dshot::Wire wire;
    gpio_num_t  gpio;
    Mode        mode{Mode::Car};
    Mounting    mounting{Mounting::Normal};
    uint8_t     poles{14};
    bool        edt{true};
    uint16_t    arm_hold_ms{1500};
    uint16_t    command_timeout_ms{250};
};

struct Rpm { int32_t value; dshot::Freshness state; };

class Esc {
public:
    explicit Esc(const Config&);

    esp_err_t begin();
    void      tick();

    void arm();
    void disarm();

    void drive(float t);            // 0..1
    bool brake(float t);            // 0..1, false when unsupported
    void coast();

    bool      direction(Direction);
    Direction direction() const;
    bool      beep(Beep);

    State            state() const;
    Rpm              rpm() const;
    dshot::Telemetry telemetry() const;
    dshot::Health    health() const;

private:
    void command(float torque);     // -1..1, negative brakes
};

}
```

The idea is that `tick()` would be called from the applications control loop (so this not having a freeRTOS task is a deliberate choice)

<!--Why do we need to state the obvious?-->
<!--`tick()` runs from the application's control loop at 200 Hz or above. There is no task in this component, because frame generation is in hardware and the sequences below need only millisecond resolution.-->

`drive()` and `brake()` call on the same function behind the scenes `command()`. Its just that its easier to work with if we have two separate functions to call from the application. We do some logic here to make it play nice with AM32.

<!--`drive()` and `brake()` reach the same actuator. AM32 cannot blend them, so both map onto one signed value and `command()` is the primitive.-->

TODO: decide what `drive()`, `brake()` and `direction()` do when a beep or configuration command is already in progress. Only one value can be on the wire at a time, so the sequence needs to own it until it finishes or is cancelled.

### Modes

<!--the `esc` NEEDS a 3d mode switch and only with that on does it behave this way, else it does not and works as a regular(regualr what? tell me what you think.) with full range.-->

<!--and for the love of god please dtop writing everythign as separate lines, a paragraph should be one line and rely on linewrap in the ide.-->
<!--AM32 with `rc_car_reverse = 1` splits the 11-bit range into two 1000-step halves. Each half's base is that half's zero point, because AM32 computes `((value - base) * 2 + 47) - reversing_dead_band` and drives only at 47 or above. The low half is based at 48, the high half at 1048, and each spans 999 (active range?).-->

`Mode` picks how torque reaches the wire.

<!--`Mode` picks how torque reaches the wire. It has to match how the ESC is configured and `esc` has no way to check that, so getting it wrong gives you a motor that drives when you asked it to brake. [100% slop no useful at all, it states an obvious problem with no fix]-->

| Mode | Throttle range | Coast sends | `drive()` | `brake()` | `direction()` |
|---|---|---|---|---|---|
| `Default` | 48 to 2047 in one direction | DShot 0 | 48 + n | returns false, coasts | returns false |
| `ThreeD` | two halves based at 48 and 1048 | drive half base | drive base + n | opposing half, the ESC brakes and then turns the other way | works |
| `Car` | two halves based at 48 and 1048 | drive half base | drive base + n | opposing half, proportional brake | works |

`ThreeD` and `Car` put identical values on the wire and differ only in what the ESC does with the braking half. In `ThreeD` the ESC brakes and then turns itself around once it is slow enough, so holding a brake there ends up as reverse. In `Car` it brakes proportionally and holds, and it only changes direction when we run the reversing sequence below.

On AM32, `ThreeD` is `bi_direction = 1` and `Car` is `rc_car_reverse = 1`.

<!--On AM32 the settings are `bi_direction = 1` for `ThreeD` and `rc_car_reverse = 1` for `Car`. Car mode switches `bi_direction` on by itself and adds the proportional brake path on top. [slop, why mention this? this is a description for a refactor why do you mix in this?]-->

### Split range mapping

This applies to `ThreeD` and `Car`. Each half's base is that half's zero point, because AM32 computes `((value - base) * 2 + 47) - reversing_dead_band` and drives only at 47 or above. The low half is based at 48 and the high half at 1048, and each has 999 usable steps above its base. The table below calls those steps `n`, which is what `drive()` and `brake()` scale onto.

`Mounting` fixes which half is that motor's forward and is set once per build. `Direction` picks which half currently drives. The other half is the brake half.

| Torque | Sent | ESC behaviour in `Car` |
|---|---|---|
| drive, `n = 1 .. 999` | drive base + n | drives |
| zero | drive base | intended coast; verify the actual FET state with our AM32 settings |
| brake, `n = 1 .. 977` | brake base + n | proportional brake, duty `n * 2 + 46` out of 2000 |
| brake, `n = 978 .. 999` | brake base + n | full brake, hard short |

The line between the two brake rows moves with `reversing_dead_band`, since AM32 switches to a hard short once the duty it computes reaches 1999.

To coast we send the drive half's base value, not DShot 0. Sending DShot 0 while the ESC is braking sets its `return_to_center` flag, which is what lets the next command flip its direction. That makes DShot 0 the one value that can change direction behind our back, so only the sequences below are allowed to send it.

<!--I feel taht the follwoing part is improtant but it is unreadable becuase it uses such contrived language. -->
<!--Coast is the drive half's base rather than DShot 0. DShot 0 sets AM32's `return_to_center` and is the only value that can change direction, so it is emitted only by the sequences below.-->

### Sequences

Four operations hold a value and then move on. Some requirements are durations, others are minimum numbers of frames. We need both. A command needing six frames still needs six frames when the rate changes, while a 1500 ms arming hold stays 1500 ms. Timing starts when the driver applies the value, not when `set()` is called. Sending six frames also does not prove the ESC received six, so use the reply where there is one and allow retries.

**Arming** holds DShot 0 in every mode. AM32 needs about a second of zero throttle after it has locked onto the signal, and that lock takes a variable few seconds. Telemetry coming back is the evidence of lock, so the hold is timed from the first reply, with `arm_hold_ms` as the hold and a timeout for a link that never answers. TODO: regular DShot has no replies, so it needs a separate startup hold based on the ESC firmware. Do not wait for telemetry in that mode.

**Configuring** sends DShot command 13 at least six times to enable extended telemetry, and the ESC answers with a version frame. Skipped when `edt` is false or the wire is not bidirectional. Decode the version reply before declaring EDT enabled, and give retries a timeout.

**Reversing** applies to the split modes. In `Car` the ESC will not switch from braking to driving the other way until a value below 48 arrives while it is braking, so we send the target half, drop below 48, then send the target half again. A motor already facing the target way passes through with no effect, which means the same sequence serves the post-arm establish and any later change. `ThreeD` turns itself around without the middle step. TODO: verify the separate sequence for that mode rather than assuming the car sequence is harmless there. In `Default` there is nothing to reverse and `direction()` returns false. Only safe at standstill, and `Esc` does not check that. The vehicle controller owns the decision and flips all four together.

**Beeping** sends command 1 through 5. AM32 handles these immediately once stopped and armed, unlike commands needing six repeats. Coasting first does not mean the motor is already stopped. TODO: wait for standstill and define the tone duration/retrigger behavior against our firmware.

### Watchdog

No call to `drive()`, `brake()` or `coast()` within `command_timeout_ms` coasts the motor rather than braking it. `esc` chooses which wire value means coast in the current mode and direction.

Checking this only in `tick()` does not cover a stalled control loop. The timer ISR should also check a deadline and use the fallback value when it expires. That deadline must come from application commands; sending another frame or receiving telemetry must not refresh it. For a sequence, define how long it may keep owning the wire before it also expires.

`Health::replies` tells us whether the ESC is answering. It cannot tell us whether the control loop is alive. A continuous transmitter could keep sending the last throttle and get perfectly good replies forever.

With one-shot transmissions, a stalled timer ISR stops starting new frames, so we also stop refreshing the ESC's signal timeout. The actual stop behavior and timeout still depend on the ESC firmware. A CPU hang needs the hardware watchdog; our ISR cannot check its own deadline while it is not running.

## Frame generation

The proposed first implementation is one GPTimer for all active links and one RMT TX/RX pair per bidirectional wire. DShot600 at 4 kHz gives a 250 us period. Four wires run together, so we do not multiply the wire time by four.

The timer callback runs the short frame-start work directly in interrupt context. It does not wake a task to call `rmt_transmit()`. Setup can use IDF for the timer, GPIO and interrupt allocation; the RMT frame path uses `hal/rmt_ll.h` and the channel memory directly. No DMA.

### Starting a frame

`set()` stores the latest request. At the next timer alarm, the driver checks command expiry and takes a coherent snapshot of each pending value. Only changed values need their sixteen symbols rebuilt. RMT must have finished its previous TX before we touch that channel's memory. Each one-shot frame has sixteen data symbols and an end marker.

Before enabling any output, finish or abort the previous RX window and check that enough time has passed since the actual previous transmission. A late callback must not cause a shortened receive window or a burst of catch-up frames. If we missed the start deadline or a channel is still busy, count it and skip that slot. Recover to the next scheduled slot with enough separation from the last real TX.

Once all active channels are ready, restore push-pull output at the correct idle level and start them using RMT's simultaneous-start support. That makes the four frames start together even though the CPU writes their start bits one at a time. Do not include an inactive channel in the synchronization mask, or the others can wait for it forever.

The timer sets the intended cadence. Interrupt latency still moves the actual frame start, so this is not zero jitter. RMT generates the bit timings once started. Measure the frame-start jitter and missed slots under the rest of the application's load.

### Handing the wire over

| Event | Work |
|---|---|
| GPTimer alarm | check deadlines, clean up the last receive window, load changed frames while TX is stopped, start active links together |
| RMT TX done | disable the completed pads' output drivers first, then reset and enable their receivers |
| RMT RX done | give RAM ownership to the CPU as required, read symbols, decode and timestamp the reply, leave the pad as an input |
| No reply before the next slot | abort RX and count `no_reply`; do not leave an old capture running into our next TX |

A bad or truncated reply still needs the same cleanup. An RX that fills its memory needs an error path too. RMT starts counting at the first edge, so its idle threshold alone cannot time out a completely silent ESC.

The critical deadline is from the end of TX until the ESC starts driving. Release all completed pads before doing any decoding. With synchronized equal-length frames, several TX-done bits should be pending together, but handle the actual status rather than assuming they always arrive in one CPU interrupt.

Keep the handler, its callees and lookup data in internal memory, and allocate an IRAM-safe interrupt. Put the timer and RMT handlers on the same chosen core and define their priorities so a frame-start handler cannot interrupt the handover halfway through. No logging, heap allocation or waiting for a task from the frame path. Keep cross-core locks short; putting a function in IRAM does not stop another critical section from delaying it.

TODO: measure worst-case TX-done-to-output-disable latency while IMU, RC, logging and any radio/flash work run. The EDT convention is 30 us turnaround, but our earlier bench notes report about 23 us on AM32. Use the earliest observed/allowed reply to set the release deadline, not a universal 30 us assumption.

## Timings

Use bitrate and RMT counter frequency to derive the symbol durations. Here a count is one RMT counter period, not a FreeRTOS tick.

```text
B = DShot bitrate in bits/s
F = RMT counter frequency in counts/s

N  = round(F / B)
A0 = round(3 * F / (8 * B))
A1 = round(3 * F / (4 * B))

zero: active for A0 counts, inactive for N - A0 counts
one:  active for A1 counts, inactive for N - A1 counts
```

Regular DShot is active high and idles low. Bidirectional DShot inverts the levels and the TX checksum. The active fractions are 3/8 for zero and 3/4 for one. Round the total once and subtract to get the other duration, so a zero and a one take exactly the same time. Use 64-bit integer intermediates; for positive integers `round(a / b)` can be `(a + b / 2) / b`.

At 40 MHz:

| Protocol | Bit counts | Zero active / inactive | One active / inactive | 16-bit frame |
|---|---|---|---|---|
| DShot150 | 267 | 100 / 167 | 200 / 67 | 106.8 us |
| DShot300 | 133 | 50 / 83 | 100 / 33 | 53.2 us |
| DShot600 | 67 | 25 / 42 | 50 / 17 | 26.8 us |
| DShot1200 | 33 | 13 / 20 | 25 / 8 | 13.2 us |

DShot1200 is not ruled out by the S3 clock. At 40 MHz its shortest nominal active pulse is 12.5 counts, so rounding to 13 gives 4% error on that pulse. At 80 MHz it gets the same 67-count bit and 25/50-count active durations that DShot600 gets at 40 MHz. Use 80 MHz if we implement it. TODO: confirm support in the installed ESC firmware and check the edges on the actual wiring.

For a requested frame rate:

```text
period_us = 1000000 / frame_hz
frame_us  = 16 * N * 1000000 / F

reply_bitrate    = 5 * B / 4
reply_bit_us     = 1000000 / reply_bitrate
reply_bit_counts = 4 * F / (5 * B)
reply_us        = 21 * reply_bit_us
```

These are ratios, not instructions to truncate every intermediate. When decoding a captured run of `d` counts, start with `round(d * 5 * B / (4 * F))` and reject runs outside the chosen timing tolerance. The ESC has its own clock, so the tolerance needs to allow its error as well as ours.

RX idle detection must be longer than any valid uninterrupted run in a reply. Start with six reply-bit periods as a conservative candidate, then confirm against the GCR decoder and captures. The noise filter must stay below the shortest valid pulse. On S3 the RX filter counts use `rmt_sclk`, while captured durations and the idle threshold use the channel counter clock; this matters if we divide those clocks differently.

The return delay comes from the ESC, not the bitrate. For scheduling, budget the latest reply start, its full driven interval, RX idle detection and software margin. For releasing our output, budget against the earliest reply start. The two bounds answer different questions.

### Continuous-loop gap, if we use it

```text
period_counts = round(F / frame_hz)
frame_counts  = 16 * N
gap_counts    = period_counts - frame_counts
```

Reject a negative gap or a period too short for the reply. Split the gap into nonzero duration fields of at most 32767 counts. Two RMT words contain four such fields, giving up to 3.2767 ms of gap at 40 MHz or 1.63835 ms at 80 MHz. So with two gap words the minimum rate is around 300 Hz or 600 Hz, depending on frame duration. Reserve another word for the zero-duration end marker: sixteen frame words plus two gap words plus the marker is nineteen words.

The TRM also gives a minimum duration immediately before an end marker in continuous mode: `10 * T_apb + 19 * T_rmt_sclk < duration * T_counter`. A long gap normally covers this, but keep its last duration long enough when splitting it. Measure the actual loop period to check any wrap overhead.

## Telemetry decode

The reply occupies 21 bit periods at 5/4 the DShot bitrate. Decode the line transitions and the four 5-bit GCR groups, then check the checksum. At DShot600 the nominal reply takes 28 us.

After GCR decode and checksum validation the payload is 12 bits.

| Condition | Meaning |
|---|---|
| `value == 0x0FFF` | ESC reports stopped |
| `(value & 0x100) == 0 && (value & 0xE00) != 0` | EDT frame, type `(value >> 8) & 0x0F`, data `value & 0xFF` |
| otherwise | eRPM period, `(value & 0x1FF) << (value >> 9)` |

The EDT space comes from normalizing eRPM periods. With a nonzero exponent, the mantissa's top bit is set. The other encodings can carry telemetry instead. Reject a decoded period of zero before dividing. TODO: document whether `Erpm::value` uses actual electrical RPM or units of 100 electrical RPM; the current driver uses the latter and the name does not say so.

| Type nibble | Field |
|---|---|
| 0x2 | temperature, degrees C |
| 0x4 | voltage, 0.25 V steps; multiply by 25 for `volt_cv` |
| 0x6 | current, 1 A steps |
| 0x8, 0xA | debug |
| 0xC | stress level, EDT v2 |
| 0xE | status or enable/version reply, interpreted with the EDT state |

AM32 interleaves current, temperature and voltage with eRPM. Its scheduler uses divisors of 40, 200 and 200 in the source checked below, but skips an extended slot after an extended reply, so these are not exact every-N-wire-frame promises. Do not use those intervals as proof that a sensor is present or a reply is missing.

Each field gets its own timestamp. An EDT reply must not refresh the last eRPM timestamp. A getter must return the value and its timestamp from the same update; `volatile` alone does not make the whole struct coherent across a task and ISR. Use a short protected snapshot or another explicitly synchronized scheme, with wrap-safe timestamp arithmetic.

`Freshness` separates a valid reading, an ESC-reported stop and no usable recent reading. A silent ESC must not become a stopped wheel. TODO: decide whether one bad reply immediately changes eRPM to `NoData` or whether the last good value stays valid until its age limit. The age limit also applies to `Stopped`. That code reports the ESC's state, not an independent measurement proving the wheel cannot be moving.

## Rate

### Wire time, bidirectional

These are planning numbers using nominal bitrate, 30 us turnaround and six reply-bit periods for RX idle detection. They do not include ISR work, additional ESC output padding or clock tolerance. The quantized TX lengths are in the timing table above.

| Protocol | Frame | Turnaround | Reply | Idle detect | Total | At 4 kHz | At 8 kHz |
|---|---|---|---|---|---|---|---|
| DShot150 | 106.7 us | 30 us | 112 us | 32 us | 280.7 us | does not fit | does not fit |
| DShot300 | 53.3 us | 30 us | 56 us | 16 us | 155.3 us | fits, about 95 us left | does not fit |
| DShot600 | 26.7 us | 30 us | 28 us | 8 us | 92.7 us | fits, about 157 us left | fits, about 32 us left |
| DShot1200 | 13.3 us | 30 us | 14 us | 4 us | 61.3 us | fits | fits |

DShot600 is the starting point because it leaves room at 4 kHz and lets us try 8 kHz without changing protocol. DShot300 can meet 4 kHz. DShot150 cannot meet the four-bidirectional-links-at-4-kHz requirement under these assumptions, though it can still be supported at lower rates. Regular DShot has no reply window and needs its own minimum inter-frame gap.

`begin()` should reject rates that cannot fit the configured timing budget. Passing that check does not prove the CPU can service the interrupts in time. Acceptance means sustaining four links at 4 kHz with no missed scheduled starts during the defined application workload. Count missed slots so we can tell when we fail that test.

### CPU cost

The previous CPU tables were estimates, not measurements. They are not a good reason to reject the timer or IDF driver yet.

At 4 kHz there are 4000 group starts and up to 16000 replies per second. At 8 kHz those numbers double. There can be one timer callback, four TX-done channel events and four RX-done channel events per group period. Several RMT events can share a CPU interrupt entry, but do not assume every set of four replies arrives together.

Measure total ISR cycles, worst-case pad-release delay, frame-start jitter, reply success rate and missed starts. A measured 10 us of total CPU work per four-link period would be 4% of one core at 4 kHz and 8% at 8 kHz. That is an example budget calculation, not a prediction of this implementation.

## Peripheral budget

One bidirectional link needs one TX-capable channel and one RX-capable channel. The S3 has exactly four of each, with 48 words per channel, so four links fit without borrowing another channel's memory.

| Chip | RMT channels | Maximum bidirectional pairs |
|---|---|---|
| ESP32 | 8, either role | 4 |
| ESP32-S2 | 4, either role | 2 |
| ESP32-S3 | 4 TX + 4 RX, fixed roles | 4 |
| ESP32-C3 | 2 TX + 2 RX | 2 |
| ESP32-C6 | 2 TX + 2 RX | 2 |
| ESP32-H2 | 2 TX + 2 RX | 2 |

This table is a channel budget, not a claim that one register implementation works on every chip. Start with S3. Use the SOC capability macros where they fit, then check the clock, memory and interrupt differences before adding other targets.

The proposed timer is shared by all links. It uses one GPTimer, not four. Link allocation and destruction change the active channel mask at a stopped boundary so the timer never uses an object after `end()` returns.

## Wiring and switching protocol

The first implementation should work with the existing signal wires and common ground, without fitting a pull-up per wire. Use push-pull to generate our frame, disable both output transistors while the ESC replies, and enable the ESP's internal pull-up to hold the idle level during that gap. This assumes the ESC actively drives both levels of its reply. TODO: verify the installed ESC target and any components already on the 4-in-1 signal inputs.

| Wire mode | While transmitting | Between frames / while receiving |
|---|---|---|
| Regular DShot | push-pull, active high | driven low, no bidirectional RX |
| Bidirectional DShot | push-pull, active low | output disabled, input enabled, internal pull-up holds high |

At setup, route RMT TX and RX to the same GPIO, leave the input buffer enabled, and select GPIO software control of output enable (`GPIO_FUNCn_OEN_SEL = 1`). Then the handover only needs the output-enable register to be cleared. `gpio_ll_output_disable()` / `gpio_ll_output_enable()` use the GPIO write-one-to-clear/set registers. With several completed pads in the same GPIO bank, their mask can be cleared in one write. There is no need to rebuild the GPIO matrix routing for every frame.

Make sure the frame ends high before releasing a bidirectional pad, and make sure RMT is idle at the intended level before enabling it again. Leave the output disabled after RX done; the next scheduled frame takes it back. A line reading high alone is not proof the ESC has released it, since it also drives high inside a reply. Use the receive window and elapsed-time checks.

### Why the internal pull-up may be enough

The S3's internal pull-up is about 45 kOhm typical. That is too weak to generate DShot600 edges for a permanently open-drain transmitter: with an illustrative 100 pF load, reaching 75% of 3.3 V from low takes about 6.2 us. The shortest DShot600 active pulse is only 0.625 us.

But that is not what we ask it to do here. Our push-pull output has already brought the line high before we release it. The internal pull-up holds that idle level, and the ESC drives the reply edges. So we do not need a strong pull-up just to make our transmitted rising edges fast. Check idle stability, leakage and the actual ESC's reply edges on the bench; the 100 pF example is not a measurement of our wiring.

### What RMT cannot do for the handover

The S3 GPIO matrix lists RMT's peripheral output-enable signal as constant 1. RMT finishing a frame does not automatically put the GPIO into input mode. Also, `RMT_IDLE_OUT_EN` chooses where the idle level comes from; clearing it does not release the pad. Software still has to disable the GPIO output before the ESC replies.

That means a resistor-free implementation is realistic, but it still depends on meeting the release deadline. A timer or continuous loop does not remove that deadline. True hardware-controlled release would need a different peripheral arrangement or external hardware; it is not an RMT option we have overlooked.

An external series resistor and an external pull-up do different things. A series resistor can limit contention current and can stay fitted in both protocols. A pull-up also does not force a wiring change when switching protocols, though it draws current whenever regular DShot drives low. Neither is required by the proposed API. Permanent open-drain would need a suitable pull-up for its rising edges, so it is not the default approach here. It also would not eliminate contention if we pulled low while the ESC was still driving high.

### Switching in software

Do the switch while disarmed. Stop scheduled starts, finish the current TX/RX exchange, disable the outputs, then change polarity, checksum behavior, pulls and RX allocation together. Reconfigure the active group consistently, restart with stop/arming frames, and enable EDT again when appropriate. The same physical wires serve both modes.

The ESP being able to switch does not guarantee the ESC can switch while already running. AM32 detects signal polarity during its startup/disarmed behavior, and the exact firmware may need signal re-detection or a power cycle. TODO: confirm this on the installed firmware and decide whether the API is `end()` plus reconfiguration or a separate operation. Do not assume changing a live `Wire` object is enough.

## Alternatives considered

### RMT continuous mode

This still has a reason to exist: RMT can repeat a frame and gap without a timer callback starting each period. An end marker wraps to the first symbol when continuous mode is enabled. CPU scheduling does not set the start of each hardware loop, though the actual period still depends on quantized counts and any loop overhead.

The hard part is changing a frame while the channel keeps reading its memory. The checksum does not make a mixed frame safe. With the bidirectional checksum and telemetry bit clear, throttle 48 is `0x0609` and throttle 87 is `0x0AEB`. Taking the first ten bits of the old frame and the last six bits of the new one produces `0x062B`: a valid throttle 49 frame.

Reading `mem_raddr_ex_chn` before and after a write does not prove we avoided a crossing. The channel can wrap between those checks, and finding out afterward cannot retract the frame. `set()` should still publish into ordinary RAM. Only a driver context with a proven safe write window should copy into channel memory. If that window is missed, keep the old complete frame for another period.

The proposed threshold interrupt after symbol 16 still needs testing. The TRM documents the threshold and continuous looping, but does not clearly establish that a threshold of 16 fires at the same place on every short loop. Check recurrence, counter reset behavior and whether the final symbol has actually finished before using the interrupt to release the pad.

We would also need a path back to push-pull before the next frame, including when no reply arrives. Re-enabling after RX done is possible for a good reply, but does not cover a silent ESC by itself. Stale-command handling must still work if the application task stops calling `tick()`.

Keep the RMT-176 idle-level workaround when stopping a continuous transmission: use the configured idle level rather than relying on the end marker's level. TODO: only prefer this implementation if the timing experiment works and the measured benefit justifies the extra update and handover logic.

### GPTimer plus the IDF RMT driver

This is also worth measuring. A timer can wake one task to submit all four frames with the IDF driver and its synchronization manager. It gives us channel allocation and transaction handling, at the cost of task scheduling and driver work on every period.

In our IDF 5.5.1, `rmt_transmit()` uses task queue operations, so do not move it into the timer ISR and assume it is safe there. The proposed direct-register path avoids that task hop by doing a bounded amount of RMT work in the callback. It does not require replacing the GPTimer driver too.

A full queue returns an error; it is not inherently a silent dropped frame. Handle the errors and measure the timing before deciding this approach is too expensive. The direct-register approach is the current preference because it gives us stopped-memory updates and a short, explicit TX/RX path.

### A container owning every link

There is no need for the application to construct a separate container just to use one motor. There is a reason for shared state inside the library: the clock, channel allocations, timer, synchronized starts and interrupt lifetime belong to the peripheral as a whole.

Keep per-wire `Link` objects and a private shared owner. TODO: specify when links can join/leave an active group and how a stopped reconfiguration is synchronized with the ISR. If we later need all four new throttle values to take effect in the same frame, add an explicit group update; four consecutive `set()` calls can otherwise straddle a frame boundary.

### Other peripherals

LCD_CAM or another sampled parallel-output design would need a new transmit/receive engine, DMA buffers and software edge extraction. There is no demonstrated benefit for these four wires, which already fit the RMT hardware. The previous specific CPU estimates for LCD_CAM were not measured and have been removed.

A separate controller with programmable I/O could generate the frame and release the pin in hardware, but it adds hardware and another link between processors. That does not fit the current goal. Stay with RMT unless the bench measurements show a limit we cannot solve on S3.

## To confirm during implementation

- Record the installed AM32 version, MCU/target and input circuit. Check reply drive mode, earliest/latest turnaround, output padding and whether protocol switching needs a restart.
- Start with one DShot600 link at 4 kHz, then all four synchronized. Capture TX, replies and a marker for when the ISR disables output.
- Measure release latency, frame-start jitter, missed starts and valid replies under the actual application load. Repeat at 8 kHz if 4 kHz passes.
- Test a silent ESC, corrupted reply, RX overflow and a stalled control loop. Check that stale receivers are aborted and that the command deadline actually reaches the fallback value.
- Verify register/memory write ordering, the end marker and synchronized restart sequence against the LL implementation and captures.
- Test regular/bidirectional switching on the same wires while disarmed. Use a scope to check idle stability and edge quality; a logic analyzer alone cannot show contention current or marginal voltage levels.
- For continuous mode, prove the threshold's behavior across repeated loops and a safe update window before implementing live memory changes.
- Finish the API decisions for command application/repetition tracking, expiry, telemetry snapshot/freshness, RPM units and sequence arbitration. Check AM32 coast/brake/reverse behavior against the firmware we actually run.

## References for the hardware details

Checked against local ESP-IDF 5.5.1 and the following sources. The local research notes include bench observations; firmware-specific assumptions still need checking against the installed ESC.

- [ESP32-S3 TRM](https://documentation.espressif.com/esp32-s3_technical_reference_manual_en.pdf): GPIO output-enable selection and table 6.11-1; RMT RAM access, clocks, one-shot/continuous/simultaneous TX and RX in chapter 37.
- [S3 RMT LL, IDF 5.5.1](https://github.com/espressif/esp-idf/blob/v5.5.1/components/hal/esp32s3/include/hal/rmt_ll.h) and [GPIO LL](https://github.com/espressif/esp-idf/blob/v5.5.1/components/hal/esp32s3/include/hal/gpio_ll.h): per-channel dividers, synchronization and output-enable writes.
- [IDF RMT group management](https://github.com/espressif/esp-idf/blob/v5.5.1/components/esp_driver_rmt/src/rmt_common.c) and [TX implementation](https://github.com/espressif/esp-idf/blob/v5.5.1/components/esp_driver_rmt/src/rmt_tx.c): ownership, queues and loop handling.
- [GPTimer documentation, IDF 5.5.1](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/peripherals/gptimer.html): ISR callbacks and cache-safe configuration.
- [ESP32-S3 datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf): internal pull-up strength and electrical characteristics.
- [RMT-176 erratum](https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32s3/03-errata-description/index.html#rmt-176-the-idle-state-signal-level-might-run-into-error-in-rmt-continuous-tx-mode): idle-level workaround.
- [DShot timings](https://betaflight.com/docs/development/API/Dshot) and [EDT specification](https://github.com/bird-sanctuary/extended-dshot-telemetry): bit fractions, reply format, command 13 and turnaround convention.
- [AM32 DShot source at 79af09b](https://github.com/am32-firmware/AM32/blob/79af09b00c52021895cddff4080a983aa1b1279c/Src/dshot.c): command repeats, immediate beep handling, polarity detection and telemetry scheduling. This is the source revision inspected, not a claim about which firmware is installed on the car.
- [Existing handover research](research/dshot-pad-damage.md): includes the earlier approximately 23 us turnaround observation.
