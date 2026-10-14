# SD card wiring: mode, bus width, and which pins are left

Answers the pin-budget question for putting an SD card on this board.
Written 2026-08-22. Pin availability is derived from `main/main.cpp`,
`components/rgb_led/rgb_led.cpp` and `sdkconfig` as they stand.

## The three modes, and why "SDIO" is the wrong word

An SD card can be talked to three ways. They are genuinely different protocols,
not marketing names for the same thing.

| Mode | Host peripheral | Signal pins | Wires |
|---|---|---|---|
| SPI mode | any GP-SPI | CS, SCK, MOSI, MISO | **4** |
| SD mode, 1-bit | SDMMC | CLK, CMD, DAT0 | **3** |
| SD mode, 4-bit | SDMMC | CLK, CMD, DAT0-DAT3 | **6** |

Two corrections worth making up front.

**"SDIO" means something else.** SDIO is the standard for non-memory peripherals
that live on an SD bus, such as WiFi cards. What you want is *SD mode*, driven by
the ESP32's **SDMMC** peripheral. This matters when reading docs, because
ESP-IDF's `sdmmc_host` driver is the one you want and its `esp_sdio` slave
support is a different thing entirely. Breakout vendors say "SDIO" loosely to
mean "native SD mode", which is where the confusion comes from.

**1-bit versus 4-bit is not the SPI-versus-SD axis.** Bus width is a property of
SD mode only. SPI mode has no width option, it is always a 4-wire full-duplex
SPI. So the three options above are the complete set, and "1-bit SPI" is not a
thing.

## The useful surprise

**1-bit SD mode needs three signal wires. SPI mode needs four.**

So for a pin-constrained board, SD mode at 1-bit is not a compromise against SPI,
it is strictly better on both axes. Fewer pins, and faster, because it drops the
SPI command framing overhead. There is no argument for SPI mode here unless the
SDMMC peripheral is already spoken for, which on this board it is not.

## Do you need 4-bit? No.

| | Throughput |
|---|---|
| Required today | ~40 KB/s |
| Required after rate-limiting `rpm` (see `logging-design.md`) | ~27 KB/s |
| 1-bit SD at 20 MHz, measured community numbers | ~1 MB/s |
| 4-bit SD at 40 MHz, measured | ~2.34 MB/s |

1-bit leaves roughly **25x headroom**. Even allowing an order of magnitude for
FATFS overhead, card stalls and a pessimistic card, it does not get close to
binding. 4-bit costs three more pins to buy speed that gets thrown away.

Start at `SDMMC_FREQ_DEFAULT` (20 MHz). `SDMMC_FREQ_HIGHSPEED` (40 MHz) exists if
ever needed, but note `input_delay_phase` in `sdmmc_host_t` only takes effect at
high speed, which is a hint that 40 MHz over the GPIO matrix is where signal
timing starts needing attention. At 20 MHz it does not.

## Pin budget on this board

ESP32-S3 has GPIO 0-21 and 26-48. On an N16R8 module, GPIO 26-32 are the SPI0/1
flash and PSRAM bus, and GPIO 33-37 are consumed by the octal PSRAM interface, so
26-37 are all off-limits. That leaves 33 usable pins.

Currently used, 21 of them:

```
1  ADC current      2  GPS TX          4  aero servo      6  aux servo 1
7  aux servo 2      9  IMU RST        10  IMU CS         11  IMU MOSI
12 IMU SCLK        13  IMU MISO       14  IMU INT        15  ADC voltage
16 RC RX           17  RC TX          38  motor RL       39  motor FR
40 motor RR        41  motor FL       42  GPS RX         47  steering servo
48 WS2812 LED
```

What is left, sorted by how safe it is to use:

| Pins | Status |
|---|---|
| **5, 8, 18, 21** | Clean. No strapping, no peripheral, no caveats. |
| 19, 20 | Native USB D-/D+. Free only if you give up USB-OTG. |
| 43, 44 | UART0 console TX/RX. Free only if you give up the serial console. |
| 0, 3, 45, 46 | Strapping pins. See the warning below. |

**Four clean pins, and 1-bit SD needs three.** It fits, with one to spare.

Suggested assignment, leaving GPIO21 free:

```
CLK  -> GPIO5
CMD  -> GPIO8
DAT0 -> GPIO18
```

Card detect is optional and not worth a pin unless you want auto-remount on
insertion. GPIO21 is the spare if you decide you do.

4-bit would need six, which forces you into the USB pins or the strapping pins.
That is the real reason not to do it, more than the throughput argument.

## Warning: never put a pulled-up SD line on GPIO45 or GPIO46

SD data and command lines carry 10 kΩ pull-ups by requirement. Two of the
remaining strapping pins react badly to being held high at reset.

**GPIO45 selects VDD_SPI voltage. Low means 3.3 V (the default), high means
1.8 V.** A 10 kΩ pull-up on GPIO45 tells the chip to run its flash and PSRAM rail
at 1.8 V. On a 3.3 V module that is a board that does not boot.

One blog got this polarity backwards while researching this, so to be explicit,
the Espressif source is the VDD_SPI Voltage Control table: `EFUSE_VDD_SPI_FORCE`
0 with GPIO45 at 0 gives 3.3 V. On a WROOM-1 the `VDD_SPI_FORCE` eFuse is
usually burned at the factory, which makes GPIO45 stop mattering, but that is not
something to bet a board on.

**GPIO46 controls ROM boot message output** and GPIO0 low at reset enters
download mode. GPIO3 selects the JTAG signal source.

None of these are fatal if you know about them, but a pulled-up SD line is
exactly the kind of thing that sets a strapping pin without anyone deciding to.
Stay on 5, 8, 18 and 21 and the question never comes up.

## The Adafruit breakout: yes, and it is the right choice

The [Adafruit Micro SD SPI or SDIO Card Breakout Board (4682)](https://www.adafruit.com/product/4682)
works at 1-bit and is well suited to it.

It breaks out CLK, CMD, D0, D1, DAT2, D3, plus 3V, GND and DET, and it carries
**pull-up resistors on all the SDIO lines**, not just the ones you connect. That
detail is what makes 1-bit clean, because the SD spec and
[ESP-IDF's pull-up requirements](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/sd_pullup_requirements.html)
both say CMD and DAT0 through DAT3 need 10 kΩ pull-ups
"regardless of whether these lines are connected to the host", so that unused
lines do not float into a wrong state. DAT3 in particular wants to be held high,
because a card sees CS asserted low at CMD0 as a request to enter SPI mode.

The consequence for your pin budget is the good part: **DAT1, DAT2 and DAT3 need
pull-ups, not GPIOs.** Leave those three pads unconnected. The board pulls them
up. You wire three signals and two power pins.

It is 3.3 V only with no level shifters, which is correct for the S3.

Note the DET pin is pulled up through 4.7 kΩ rather than 10 kΩ, and it is a card
detect line rather than a bus line, so it does not participate in any of the
above.

## The "share the IMU's SPI bus" alternative, and why not

Worth addressing because it is the minimum-pin option and it is tempting.

Both general-purpose SPI peripherals are already taken. SPI2 runs the BNO08x
(`main.cpp`, `host = SPI2_HOST`) and SPI3 runs the WS2812 strip
(`rgb_led.cpp`, GPIO48). There is no third GP-SPI to route through the GPIO
matrix. So "route my last SPI to unused pins" would actually mean hanging the SD
card off SPI2 alongside the IMU, sharing MOSI, MISO and SCLK and adding one CS
pin.

That costs **one** GPIO instead of three, which is genuinely attractive. The
reason to decline anyway:

- ESP-IDF serialises transactions on a shared bus. A 4 KB SD write at 20 MHz
  holds the bus for something like 1.6 ms plus card latency, and SD cards
  occasionally stall for tens of milliseconds during internal wear levelling.
- The BNO08x is polled at 200 Hz, one sample every 5 ms, and it is
  interrupt-driven off the HINT line. Blocking its bus access for milliseconds at
  unpredictable times adds jitter to the sensor the control loop depends on, and
  a card stall could drop samples outright.
- The IMU runs at 2 MHz and the card wants 20 MHz. Per-device clock config
  handles that fine, but it is one more thing sharing a bus that already had a
  difficult bring-up (see the BNO08x notes in memory).

Spending two extra pins to keep the SD card on a completely separate peripheral,
with its own DMA and no interaction with the IMU, is a good trade when you have
four pins free.

## Configuration sketch

```cpp
sdmmc_host_t host = SDMMC_HOST_DEFAULT();
host.flags     = SDMMC_HOST_FLAG_1BIT;      // 1-line mode
host.max_freq_khz = SDMMC_FREQ_DEFAULT;     // 20 MHz, plenty

sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
slot.width = 1;
slot.clk = GPIO_NUM_5;
slot.cmd = GPIO_NUM_8;
slot.d0  = GPIO_NUM_18;
slot.cd  = SDMMC_SLOT_NO_CD;                // no card-detect pin wired
slot.wp  = SDMMC_SLOT_NO_WP;
// no internal pullup flag, the breakout has real 10k pull-ups
```

Per-signal GPIO assignment works because the S3 routes SDMMC through the GPIO
matrix (`CONFIG_SOC_SDMMC_USE_GPIO_MATRIX=y` in `sdkconfig`), and Espressif's
SDMMC docs state plainly that on the S3 "any GPIO may be used for each of the SD
card signals". This is not true of the original ESP32, where slot 1 was pinned to
fixed IO MUX pins, so older wiring guides do not apply.

## Summary

- Use **SD mode at 1-bit**, not SPI mode. Three wires instead of four, and faster.
- **Three signal pins**: GPIO5 CLK, GPIO8 CMD, GPIO18 DAT0. GPIO21 spare.
- Leave DAT1, DAT2, DAT3 unconnected. The breakout's pull-ups handle them.
- Skip 4-bit. 1-bit already has 25x the throughput this system needs.
- Do not let a pulled-up SD line land on GPIO45 or GPIO46.
- Do not share the IMU's SPI bus, even though it would cost only one pin.

## Sources

- [SDMMC Host Driver, ESP32-S3, ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/sdmmc_host.html)
- [SD Pull-up Requirements, ESP32-S3, ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/sd_pullup_requirements.html)
- [ESP32-S3 Series Datasheet, VDD_SPI Voltage Control](https://documentation.espressif.com/esp32-s3_datasheet_en.pdf)
- [Adafruit MicroSD SPI or SDIO Card Breakout, pinouts](https://learn.adafruit.com/adafruit-microsd-spi-sdio/pinouts)
- [Adafruit product 4682](https://www.adafruit.com/product/4682)
- [ESP32-S3 pinout quick guide, atomic14](https://github.com/atomic14/esp32-s3-pinouts)
- Local: `sd_protocol_types.h` and `sdmmc_host.h` in `~/esp/esp-idf-v5.5.1`
