"""Bidirectional DShot HLA for Saleae Logic 2.

Input is an Async Serial analyzer running at 3x the DShot bit rate with 48 bits
per frame, which oversamples each DShot bit three times. DShot encodes bits as
pulse width, so a 1:1 UART mapping cannot recover them; with 3x sampling the
sample at wire index 3n lands mid-bit and separates a short pulse from a long
one.

Logic 2 hands a 48-bit frame over as 6 bytes, big-endian over a value whose bit
i is wire bit i, so wire bit i is bit i of int.from_bytes(data, "big").

At mid-bit the line always reads as the complement of the data bit, in both
polarities, provided Async Serial's Inverted flag is set to match the idle
level. So the sample is inverted unconditionally and the Polarity setting only
selects the CRC form.
"""

from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame, ChoicesSetting

DSHOT_COMMANDS = {
    0: "MOTOR_STOP", 1: "BEEP1", 2: "BEEP2", 3: "BEEP3", 4: "BEEP4", 5: "BEEP5",
    6: "ESC_INFO", 7: "SPIN_DIR_1", 8: "SPIN_DIR_2", 9: "3D_MODE_OFF",
    10: "3D_MODE_ON", 11: "SETTINGS_REQUEST", 12: "SAVE_SETTINGS",
    13: "EDT_ENABLE", 14: "EDT_DISABLE", 20: "SPIN_DIR_NORMAL",
    21: "SPIN_DIR_REVERSED", 36: "ENTER_PROGRAMMING", 37: "PROGRAM_COMMIT",
}


class Hla(HighLevelAnalyzer):
    polarity = ChoicesSetting(
        label="Polarity",
        choices=["Bidirectional (idle HIGH, inverted CRC)",
                 "Standard (idle LOW, normal CRC)"],
    )
    split_range = ChoicesSetting(
        label="Throttle range",
        choices=["3D / car mode (48-1047 + 1048-2047)", "Normal (48-2047)"],
    )

    result_types = {
        "throttle": {"format": "{{data.label}}  crc={{data.crc}}"},
        "command":  {"format": "CMD {{data.name}} ({{data.value}}) telem={{data.telem}}  crc={{data.crc}}"},
        "stop":     {"format": "DShot 0 (stop/neutral)  crc={{data.crc}}"},
        "esc_telem": {"format": "ESC telemetry response"},
    }

    def __init__(self):
        self.inverted_crc = "Bidirectional" in str(self.polarity)
        self.split = "3D" in str(self.split_range)
        self.last_ok_end = None

    def decode(self, frame: AnalyzerFrame):
        if frame.type != "data":
            return None
        d = frame.data.get("data")
        if not isinstance(d, (bytes, bytearray)) or len(d) != 6:
            return None

        v = int.from_bytes(bytes(d), "big")

        word = 0
        for n in range(16):
            word = (word << 1) | (1 - ((v >> (3 * n)) & 1))

        value = word >> 5
        telem = (word >> 4) & 1
        got = word & 0x0F
        payload = word >> 4
        exp = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0F
        if self.inverted_crc:
            exp = (~exp) & 0x0F
        crc = "OK" if got == exp else f"FAIL exp{exp:X} got{got:X}"

        st, et = frame.start_time, frame.end_time

        # Async Serial also frames on the ESC's telemetry burst, which is not
        # DShot and decodes to noise. It lands in the turnaround window just
        # after a good frame, so mark it rather than reporting a bogus throttle.
        # Its presence is useful on its own: the ESC only answers once it has
        # locked onto the signal (inputSet == 1), which happens before arming.
        if got != exp and self.last_ok_end is not None:
            try:
                gap = float(st - self.last_ok_end)
            except Exception:
                gap = -1.0
            if 0.0 < gap < 150e-6:
                self.last_ok_end = None
                return AnalyzerFrame("esc_telem", st, et, {})

        if got == exp:
            self.last_ok_end = et

        if value == 0:
            return AnalyzerFrame("stop", st, et, {"crc": crc})
        if value < 48:
            return AnalyzerFrame("command", st, et, {
                "value": value, "name": DSHOT_COMMANDS.get(value, "?"),
                "telem": telem, "crc": crc})

        if self.split:
            base, half = (48, "LOW") if value <= 1047 else (1048, "HIGH")
            step = value - base
            label = f"{half} {value} (base+{step}, {100.0 * step / 999:.1f}%)"
        else:
            label = f"THR {value} ({100.0 * (value - 48) / 1999:.1f}%)"

        return AnalyzerFrame("throttle", st, et, {
            "label": label, "value": value, "telem": telem,
            "raw": f"{word:04X}", "crc": crc})
