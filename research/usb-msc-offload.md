# USB drive offload, and whether flashing still works

The idea: plug in a cable, the car shows up as a drive, drag the log files off.
The requirement: `idf.py flash` still has to work without extra steps.

Written 2026-08-22, updated after learning the board has two USB-C ports.

## Answer

Yes. The two ports solve it. Flash over COM, run the drive on the USB port.
They are separate hardware and do not fight.

| Port | What is behind it | Used for |
|---|---|---|
| **COM** | CH343P USB-to-serial chip, wired to UART0 on GPIO43 and GPIO44 | Flashing |
| **USB** | The ESP32-S3's own USB, GPIO19 and GPIO20 | Free for MSC |

Flashing over COM goes through the CH343P. Its DTR and RTS lines are wired to EN
and GPIO0, so esptool resets the chip into download mode using real hardware
lines. That never touches the ESP32-S3's USB peripheral. Nothing in the project
uses GPIO19 or GPIO20.

Two USB-C ports labelled COM and USB, with 16 MB flash and 8 MB octal PSRAM,
means a YD-ESP32-S3 N16R8 or a copy of its layout. That matches the flash and
PSRAM figures in `codebase-overview.md`.

The only thing you give up is that the USB port can no longer be USB-Serial-JTAG
while it is a drive. The chip's two USB controllers share one PHY. That costs you
console output on that port and nothing else, because flashing was never there.

**Confirm before building:** flash with only the COM cable plugged in and the USB
cable unplugged. If that works, everything above holds.

## Is it worth building

ESP32-S3 USB is full speed, 12 Mbit/s. Real MSC throughput lands under 1 MB/s
after protocol overhead and SD access.

The fair comparison is not MSC against WiFi. It is MSC against pulling the card
and putting it in a reader, which runs at tens of MB/s.

At 40 KB/s of records, an hour of driving is about 141 MB. Over MSC that is
three to five minutes. In a USB 3 card reader it is a few seconds.

So MSC saves effort, not time. Build it if the card ends up somewhere awkward to
reach in the chassis. Skip it if you can get at the slot, because pulling the
card is faster, simpler, and has no firmware that can break.

Worth settling before writing any code.

## The one thing that will corrupt your card

MSC hands the host raw blocks. Since `logging-alternatives.md` settled on an SD
card, the card is already FAT, so MSC passes reads and writes straight through to
`sdmmc_read_sectors` and `sdmmc_write_sectors`. The host mounts the real
filesystem. ESP-IDF's `usb/device/tusb_msc` example does this and supports an SD
card as the backing store.

**Only one side can touch the card at a time.** The firmware's FATFS mount and
the host's block-level view cannot both be live. If the ESP32 writes a log record
while macOS has the volume mounted, the host's cached FAT data and the real card
diverge and the filesystem is corrupted.

So starting the drive has to mean:

1. Stop the logger and close the current file.
2. Call `esp_vfs_fat_sdcard_unmount()`.
3. Start TinyUSB MSC on the same `sdmmc_card_t`.

Leaving the drive means a reboot.

The firmware needs some signal for when to hand the card over. TinyUSB's own
mount callback is enough, optionally gated on the car being disarmed.

`sd-card-wiring.md` leaves card detect unwired (`slot.cd = SDMMC_SLOT_NO_CD`), so
MSC has to fail cleanly when no card is present rather than assuming one is
there.

Deleting old files, which was an open question for the internal-flash options, is
free here. You do it in Finder.

## Steps

1. Confirm flashing works over COM alone.
2. Decide whether the card slot is reachable. If it is, stop here.
3. Build MSC on the USB port from the `tusb_msc` example, backed by the SD card,
   with the unmount handoff above.

## Sources

- [YD-ESP32-S3 hardware documentation, dual USB-C with CH343P](https://github.com/vcc-gnd/YD-ESP32-S3)
- [YD-ESP32-23, hardware USB-serial (CH343P) plus native ESP32-S3 USB](https://github.com/rtek1000/YD-ESP32-23)
- [USB FAQ, ESP-FAQ, two USB controllers sharing one PHY](https://docs.espressif.com/projects/esp-faq/en/latest/software-framework/peripherals/usb.html)
- [tusb_msc example, ESP-IDF](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/device/tusb_msc)
