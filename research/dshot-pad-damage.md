# How bidirectional DShot killed GPIO41

Written 2026-08-24. GPIO41 was the first `motor_fl` pad. It now reads about
2.15 V with nothing attached to the board, its input buffer no longer responds
to the internal pull-up or pull-down, and it measures roughly 4.5 MOhm to
ground with the board unpowered while GPIO38, 39, 40 and 42 all measure open.
Its output driver still works, which is why the fault stayed invisible for so
long. Motors spun, only telemetry never came back.

The explanation below fits every one of those measurements. It has not been
caught in the act on a scope, so treat the mechanism as the best account of the
damage rather than as an observation of it.

## One wire, two talkers

Ordinary DShot is one-way. The ESP32 drives the signal wire and the ESC only
listens. Nothing else ever puts a voltage on that wire, so the ESP32 can drive
it hard in both directions, pushing it up to 3.3 V and pulling it down to 0 V
with a strong transistor at each end. That is called push-pull, and it is what
the RMT peripheral does by default.

Bidirectional DShot changes the arrangement. After each command frame the ESC
answers on the same wire with the motor's electrical RPM. So the wire now has
two devices that can both drive it, and each of them has the same kind of
strong push-pull output stage.

Two push-pull outputs on one wire is only safe as long as exactly one of them
is switched on at any moment. If the ESP32 is holding the wire up at 3.3 V at
the same instant the ESC is holding it down at 0 V, there is nothing between
3.3 V and ground except the two output transistors. Both are designed to be
low resistance, a few tens of ohms each. The result is a current of several
tens of milliamps flowing straight through both chips. The S3's pads are rated
for 40 mA, so this sits at or past the limit, and it happens for as long as the
two disagree.

## The handover

To avoid that, the firmware has to switch its own driver off before the ESC
starts talking, and switch it back on only after the ESC has finished. The
switch is done by putting the pad into open-drain mode. In open-drain the pad
can still pull the wire down, but it can no longer push it up. It simply lets
go, and a pull-up resistor holds the line high while nobody is driving it.

In `components/motor/dshot_driver.cpp` that is `enable_rx()`, which calls
`gpio_ll_od_enable`, and `disable_rx()`, which calls `gpio_ll_od_disable`. The
first hands the wire to the ESC. The second takes it back.

The timing is tight. At DShot600 a command frame lasts about 27 microseconds.
AM32 starts answering roughly 23 microseconds after the frame ends, and its
reply runs for about another 28 microseconds. The bench loop repeats the whole
cycle every 125 microseconds. Every handover has to land inside a window a few
tens of microseconds wide, eight thousand times a second.

## Where the handover misses

There are two moments where it can go wrong, and both are unguarded.

The first is letting go too late. `enable_rx()` runs inside the transmit-done
interrupt handler. If that interrupt is delayed, by a higher priority handler,
by a cache miss fetching code that is not in IRAM, or by anything else
competing for the core, the pad is still in push-pull mode when the ESC begins
its reply. The ESC pulls the wire down, the ESP32 is still pushing it up, and
they fight for the length of every low bit in the reply.

The second is taking the wire back too early, and this one is not a rare timing
accident. `disable_rx()` is called from `mode_tx()` on the way into every
transmission, unconditionally, whether or not the ESC has finished. The driver
already carries a comment saying receptions are not terminating on their own.
Whenever the transmit loop comes round while the ESC is still mid-reply, the
pad goes back to push-pull and drives high into an ESC that is driving low.

Either way the two chips end up in a short through their own output stages, and
at 8 kHz the opportunity comes up eight thousand times per second. Across the
hours of bench sessions in the `.sal` captures, the count is in the hundreds of
millions.

## The other stress on the same wire

Contention is not the only thing hurting the pad. The signal wire runs to an
ESC sitting next to a motor that is switching large currents, and the wire has
enough inductance to ring on fast edges. When a falling edge overshoots
downward, the pad goes below ground.

Every input pin has protection diodes to ground and to the supply rail. They
start conducting once the pin goes about 0.3 V past either rail. That is their
job, and they survive brief events fine. But the current through them is
limited only by the impedance of whatever is driving the ringing, and there is
no series resistor on this line to hold it down. Repeated for hours, this is
wear rather than a single failure.

## Why the input died and the output lived

The odd part of the symptom is that the pad can still drive a signal out but
can no longer read one in. That is the expected order of failure rather than
evidence against it.

The input buffer's transistors sit directly on the pad and are built small,
with a very thin insulating layer at the gate. That layer is what breaks down
under overvoltage and repeated clamp conduction. The output driver is built
from large transistors designed to carry tens of milliamps continuously, so it
tolerates the same abuse for far longer.

An input buffer that has broken down no longer presents a clean high impedance
to the pad. It leaks, both to ground and to the supply, which is why the pad
settles at an intermediate voltage instead of following the internal pull-down,
and why the unpowered resistance measurement finds a path to ground that the
neighbouring pins do not have.

## What this means for the current setup

`motor_fl` moved to GPIO18 and the driver code did not change, so GPIO18 is now
accumulating the same damage under the same conditions.

Two routes below, and they call for different resistors in different places.
Keeping push-pull transmit wants a resistor in series with the line and no
pull-up, because the chip drives its own rising edges and the ESC drives the
rising edges of its reply. Permanent open-drain wants a pull-up to 3.3 V,
because with the pad unable to push high a resistor has to make every rising
edge instead.

### Series resistor in each signal line

The single highest value change is a resistor of 220 to 330 ohms in series with
each motor signal wire at the ESP32 end. The S3's output stage is around
40 ohms and the ESC's is similar, so an unlimited fight between them draws
about 40 mA, at the pad's rating. With 220 ohms in the line the same fight
draws 11 mA. The same resistor damps the ringing the clamp diodes have been
absorbing.

It costs nothing elsewhere. Against a hundred picofarads of wire capacitance
220 ohms gives a 50 nanosecond edge, against a DShot600 bit of 1.67
microseconds. The ESC's reply is unaffected because the receiving input draws
no current, so there is no voltage drop across the resistor in that direction.

Check the ESC first. Many have a series resistor on the signal input already,
in which case the contention current was limited from the start and the ringing
did more of the damage than the fighting did.

### Gate the turnaround in firmware

`mode_tx()` calls `disable_rx()` on the way into every transmission with no
check on whether the receive has completed, and receives do not self-terminate.
That is the systematic collision rather than a rare timing slip, and gating it
costs nothing. Flight controllers run bidirectional DShot on ordinary push-pull
pins without destroying them, because their turnaround is honoured, not because
of anything on the board.

### Permanent open-drain, optional

Contention becomes electrically impossible if the pad never drives high at all.
Leave it in open-drain permanently, let the RMT only pull down, and no
interrupt latency can produce a fight.

The cost is an external pull-up on every line, because the chip's internal one
is around 45 kOhm and gives a rise time near 1.4 microseconds against a 1.67
microsecond bit. It also constrains the series resistor above, since the low
level then becomes a divider between the two. 330 ohms into a 1 kOhm pull-up
sits at 0.82 V, which is at the input threshold and too close to be worth
shipping. Either use a large pull-up with a small series resistor or accept
that this route replaces the series resistor rather than adding to it.

This buys little over the first two changes together, and it is four more
resistors and a constraint to remember.
