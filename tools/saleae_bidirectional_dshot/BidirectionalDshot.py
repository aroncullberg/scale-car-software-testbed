"""Saleae Logic 2 High-Level Analyzer (HLA) for Bidirectional DShot (BDshot).

Decodes:
- Forward DShot Frames (FC -> ESC): 16-bit DShot commands / throttle (DShot150/300/600/1200)
- Inverted / Bidirectional DShot CRC4 verification
- Reverse Telemetry Frames (ESC -> FC): GCR 5b/4b encoded eRPM / EDT packets
- Electrical RPM (eRPM), Mechanical RPM calculation based on configured motor poles
- Extended DShot Telemetry (EDT): Voltage, Current, Temperature, Debug
"""

try:
    from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame, StringSetting, NumberSetting, ChoicesSetting
except ImportError:
    # Allows offline import / unit testing outside Saleae Logic 2
    class HighLevelAnalyzer:
        pass
    class AnalyzerFrame:
        def __init__(self, frame_type, start_time, end_time, data=None):
            self.type = frame_type
            self.start_time = start_time
            self.end_time = end_time
            self.data = data or {}
    class ChoicesSetting:
        def __init__(self, choices=None, label="", default=None):
            self.choices = choices or []
            self.label = label
            self.default = default
    class NumberSetting:
        def __init__(self, min_value=0, max_value=100, label="", default=14):
            self.min_value = min_value
            self.max_value = max_value
            self.label = label
            self.default = default
    class StringSetting:
        def __init__(self, label="", default=""):
            self.label = label
            self.default = default


# Standard GCR (Group Code Recording) 5b -> 4b lookup table for DShot telemetry
GCR_DECODE_TABLE = {
    0x19: 0x0,
    0x1B: 0x1,
    0x12: 0x2,
    0x13: 0x3,
    0x1D: 0x4,
    0x15: 0x5,
    0x16: 0x6,
    0x17: 0x7,
    0x1A: 0x8,
    0x09: 0x9,
    0x0A: 0xA,
    0x0B: 0xB,
    0x1E: 0xC,
    0x0D: 0xD,
    0x0E: 0xE,
    0x0F: 0xF,
}

# Common DShot Special Commands
DSHOT_COMMANDS = {
    0: "DISARM / 0 Throttle",
    1: "BEEP 1",
    2: "BEEP 2",
    3: "BEEP 3",
    4: "BEEP 4",
    5: "BEEP 5",
    6: "ESC INFO",
    7: "SPIN DIR 1 (CW)",
    8: "SPIN DIR 2 (CCW)",
    9: "3D MODE OFF",
    10: "3D MODE ON",
    11: "SETTINGS REQUEST",
    12: "SAVE SETTINGS",
    13: "EXTENDED TELEM ENABLE",
    14: "EXTENDED TELEM DISABLE",
    20: "SPIN DIR NORMAL",
    21: "SPIN DIR REVERSED",
    42: "LED 0",
    43: "LED 1",
    44: "LED 2",
    45: "LED 3",
    46: "LED 4",
    47: "LED 5",
}


class Hla(HighLevelAnalyzer):
    """Bidirectional DShot High-Level Analyzer for Saleae Logic 2."""

    # UI Settings in Logic 2
    dshot_speed = ChoicesSetting(
        label="DShot Speed",
        choices=["DShot600", "DShot300", "DShot150", "DShot1200"],
    )

    mode = ChoicesSetting(
        label="Mode / Polarity",
        choices=["Bidirectional (Inverted / Idle HIGH)", "Standard (Non-Inverted / Idle LOW)"],
    )

    motor_poles = NumberSetting(
        label="Motor Poles (for RPM calculation)",
        min_value=2,
        max_value=36,
    )

    strict_crc = ChoicesSetting(
        label="CRC Validation",
        choices=["Strict (Drop invalid frames)", "Show Errors (Display failed CRCs)"],
    )

    def __new__(cls, settings=None, *args, **kwargs):
        """Auto-populates missing settings when Logic 2 reloads with a cached session config."""
        if isinstance(settings, dict):
            defaults = {
                "dshot_speed": "DShot600",
                "mode": "Bidirectional (Inverted / Idle HIGH)",
                "motor_poles": 14,
                "strict_crc": "Strict (Drop invalid frames)",
            }
            for k, v in defaults.items():
                if k not in settings:
                    settings[k] = v
        try:
            return super().__new__(cls, settings, *args, **kwargs)
        except TypeError:
            return super().__new__(cls)

    # Frame display formats in Logic 2 timeline and data table
    result_types = {
        "fc_throttle": {
            "format": "FC: Throttle={{data.throttle}} ({{data.throttle_pct}}%) [ReqTelem={{data.telem_req}}] CRC: {{data.crc_status}}"
        },
        "fc_command": {
            "format": "FC CMD: {{data.cmd_name}} ({{data.cmd_id}}) [ReqTelem={{data.telem_req}}] CRC: {{data.crc_status}}"
        },
        "esc_erpm": {
            "format": "ESC: {{data.rpm}} RPM (eRPM={{data.erpm}}, Period={{data.period_us}}µs) CRC: {{data.crc_status}}"
        },
        "esc_stopped": {
            "format": "ESC: Motor Stopped (eRPM=0) CRC: {{data.crc_status}}"
        },
        "esc_edt": {
            "format": "ESC EDT: {{data.edt_type}} = {{data.edt_value}} CRC: {{data.crc_status}}"
        },
        "esc_raw": {
            "format": "ESC Raw: 0x{{data.hex_val}} CRC: {{data.crc_status}}"
        },
        "crc_error": {
            "format": "[CRC FAIL] Raw: 0x{{data.raw_hex}} (Exp: 0x{{data.exp_crc:X}}, Got: 0x{{data.got_crc:X}})"
        },
        "parse_error": {
            "format": "Error: {{data.error_msg}}"
        },
    }

    def __init__(self):
        self.byte_buffer = []
        self.frame_start_time = None
        self.last_byte_end_time = None
        self.last_fc_end_time = None

        # Determine settings safely (compatible with both Logic 2 runtime and offline/mock test environments)
        speed_raw = getattr(self, "dshot_speed", "DShot600")
        speed_str = str(speed_raw.default if hasattr(speed_raw, "default") else speed_raw)
        if "1200" in speed_str:
            self.symbol_us = 0.833
        elif "300" in speed_str:
            self.symbol_us = 3.333
        elif "150" in speed_str:
            self.symbol_us = 6.667
        else:
            self.symbol_us = 1.667  # DShot600 default

        self.byte_gap_threshold_s = (self.symbol_us * 12) / 1e6

        mode_raw = getattr(self, "mode", "Bidirectional")
        mode_str = str(mode_raw.default if hasattr(mode_raw, "default") else mode_raw)
        self.is_bidirectional = "Bidirectional" in mode_str

        poles_raw = getattr(self, "motor_poles", 14)
        if hasattr(poles_raw, "default"):
            self.poles = int(poles_raw.default or 14)
        elif isinstance(poles_raw, (int, float)):
            self.poles = int(poles_raw)
        else:
            self.poles = 14

        crc_raw = getattr(self, "strict_crc", "Strict")
        crc_str = str(crc_raw.default if hasattr(crc_raw, "default") else crc_raw)
        self.is_strict_crc = "Strict" in crc_str

    def _decode_fc_frame(self, b0: int, b1: int, start_time, end_time):
        """Decode a 16-bit FC frame from two consecutive received bytes."""
        if self.is_bidirectional:
            # Active-low pulses sampled by non-inverted UART sampler yield inverted bits
            raw_word = ((~b0 & 0xFF) << 8) | (~b1 & 0xFF)
        else:
            raw_word = ((b0 & 0xFF) << 8) | (b1 & 0xFF)

        throttle_or_cmd = (raw_word >> 5) & 0x07FF
        telem_req = (raw_word >> 4) & 0x01
        received_crc = raw_word & 0x0F

        # Check CRC4
        payload12 = (raw_word >> 4) & 0x0FFF
        if self.is_bidirectional:
            expected_crc = (~(payload12 ^ (payload12 >> 4) ^ (payload12 >> 8))) & 0x0F
        else:
            expected_crc = (payload12 ^ (payload12 >> 4) ^ (payload12 >> 8)) & 0x0F

        crc_valid = (received_crc == expected_crc)
        crc_status = "OK" if crc_valid else f"FAIL (exp 0x{expected_crc:X}, got 0x{received_crc:X})"

        if not crc_valid:
            if getattr(self, "is_strict_crc", True):
                # Suppress corrupt frames (such as random noise or mis-sampled ESC pulses)
                return None
            return AnalyzerFrame(
                "crc_error",
                start_time,
                end_time,
                {
                    "raw_hex": f"{raw_word:04X}",
                    "exp_crc": expected_crc,
                    "got_crc": received_crc,
                },
            )

        if throttle_or_cmd < 48:
            cmd_id = throttle_or_cmd
            cmd_name = DSHOT_COMMANDS.get(cmd_id, f"Special Cmd {cmd_id}")
            return AnalyzerFrame(
                "fc_command",
                start_time,
                end_time,
                {
                    "cmd_id": cmd_id,
                    "cmd_name": cmd_name,
                    "telem_req": telem_req,
                    "raw_hex": f"{raw_word:04X}",
                    "crc_status": crc_status,
                    "crc_ok": crc_valid,
                },
            )
        else:
            throttle_val = throttle_or_cmd
            throttle_pct = f"{((throttle_val - 48) / 1999.0) * 100.0:.1f}"
            return AnalyzerFrame(
                "fc_throttle",
                start_time,
                end_time,
                {
                    "throttle": throttle_val,
                    "throttle_pct": throttle_pct,
                    "telem_req": telem_req,
                    "raw_hex": f"{raw_word:04X}",
                    "crc_status": crc_status,
                    "crc_ok": crc_valid,
                },
            )

    def _decode_esc_gcr(self, raw_20bit: int, start_time, end_time):
        """Decode a 20-bit GCR telemetry frame from the ESC."""
        # Convert NRZ-I if needed: raw_val ^ (raw_val >> 1)
        # Try both direct GCR and transition-decoded GCR
        candidates = [raw_20bit, (raw_20bit ^ (raw_20bit >> 1)) & 0xFFFFF]

        decoded_result = None
        for val in candidates:
            q3 = (val >> 15) & 0x1F
            q2 = (val >> 10) & 0x1F
            q1 = (val >> 5) & 0x1F
            q0 = val & 0x1F

            if q3 in GCR_DECODE_TABLE and q2 in GCR_DECODE_TABLE and q1 in GCR_DECODE_TABLE and q0 in GCR_DECODE_TABLE:
                n3 = GCR_DECODE_TABLE[q3]
                n2 = GCR_DECODE_TABLE[q2]
                n1 = GCR_DECODE_TABLE[q1]
                n0 = GCR_DECODE_TABLE[q0]
                decoded16 = (n3 << 12) | (n2 << 8) | (n1 << 4) | n0

                # Check checksum: inverted XOR nibbles
                csum = decoded16 ^ (decoded16 >> 8) ^ (decoded16 >> 4)
                if (csum & 0x0F) == 0x0F:
                    decoded_result = decoded16
                    break

        if decoded_result is None:
            # Fallback to direct decoding even if CRC fails to provide visibility
            q3 = (raw_20bit >> 15) & 0x1F
            q2 = (raw_20bit >> 10) & 0x1F
            q1 = (raw_20bit >> 5) & 0x1F
            q0 = raw_20bit & 0x1F
            n3 = GCR_DECODE_TABLE.get(q3, 0)
            n2 = GCR_DECODE_TABLE.get(q2, 0)
            n1 = GCR_DECODE_TABLE.get(q1, 0)
            n0 = GCR_DECODE_TABLE.get(q0, 0)
            decoded_result = (n3 << 12) | (n2 << 8) | (n1 << 4) | n0
            crc_valid = False
            crc_status = f"FAIL (GCR=0x{raw_20bit:05X})"
        else:
            crc_valid = True
            crc_status = "OK"

        erpm_raw = (decoded_result >> 4) & 0x0FFF

        if erpm_raw == 0:
            return AnalyzerFrame(
                "esc_stopped",
                start_time,
                end_time,
                {
                    "erpm": 0,
                    "rpm": 0,
                    "crc_status": crc_status,
                    "raw_hex": f"{decoded_result:04X}",
                },
            )

        if erpm_raw == 0x0FFF:
            # EDT Escape Code / Stopped state
            return AnalyzerFrame(
                "esc_stopped",
                start_time,
                end_time,
                {
                    "erpm": 0,
                    "rpm": 0,
                    "crc_status": crc_status,
                    "raw_hex": f"{decoded_result:04X}",
                },
            )

        # Extended DShot Telemetry (EDT) format check:
        # If highest nibble of 12-bit data indicates telemetry packet type
        edt_type_nibble = (erpm_raw >> 8) & 0x0F
        if edt_type_nibble in (0x02, 0x04, 0x06, 0x08, 0x0A):
            # EDT Telemetry packet
            edt_val = erpm_raw & 0xFF
            edt_names = {
                0x02: ("Temperature", f"{edt_val} °C"),
                0x04: ("Voltage", f"{edt_val / 10.0:.2f} V"),
                0x06: ("Current", f"{edt_val} A"),
                0x08: ("Debug 1", f"0x{edt_val:02X}"),
                0x0A: ("Debug 2", f"0x{edt_val:02X}"),
            }
            edt_type, edt_str = edt_names.get(edt_type_nibble, ("EDT Data", f"{edt_val}"))
            return AnalyzerFrame(
                "esc_edt",
                start_time,
                end_time,
                {
                    "edt_type": edt_type,
                    "edt_value": edt_str,
                    "crc_status": crc_status,
                    "raw_hex": f"{decoded_result:04X}",
                },
            )

        # Standard eRPM Period calculation
        exponent = (erpm_raw >> 9) & 0x07
        mantissa = erpm_raw & 0x01FF
        period_us = mantissa << exponent

        if period_us > 0:
            erpm = (1000000 * 60) // period_us
            pole_pairs = max(1, self.poles // 2)
            mech_rpm = erpm // pole_pairs
        else:
            erpm = 0
            mech_rpm = 0

        return AnalyzerFrame(
            "esc_erpm",
            start_time,
            end_time,
            {
                "rpm": mech_rpm,
                "erpm": erpm,
                "period_us": period_us,
                "crc_status": crc_status,
                "raw_hex": f"{decoded_result:04X}",
            },
        )

    def decode(self, frame: AnalyzerFrame):
        """Processes incoming bytes from the low-level Async Serial analyzer."""
        if frame.type != "data":
            return None

        data_val = frame.data.get("data")
        if data_val is None:
            return None

        # Convert data_val to integer byte
        if isinstance(data_val, (bytes, bytearray)):
            byte_val = data_val[0]
        elif isinstance(data_val, int):
            byte_val = data_val
        elif isinstance(data_val, str):
            byte_val = ord(data_val[0])
        else:
            return None

        current_start = frame.start_time
        current_end = frame.end_time

        # Calculate gap from last received byte
        gap = 0.0
        if self.last_byte_end_time is not None:
            try:
                gap = float(current_start - self.last_byte_end_time)
            except Exception:
                gap = 0.0

        out_frames = []

        # Inter-byte gap threshold (time between distinct packet bursts)
        # At DShot600 (symbol=1.67µs), 2 consecutive bytes inside the same burst have ~0µs gap.
        # The turnaround gap between FC and ESC is ~25-40µs (>15µs).
        # The loop idle gap between ESC and next FC is >80µs.
        burst_gap_threshold_s = (self.symbol_us * 8) / 1e6

        # If a significant gap occurred since the last byte, flush any pending buffer
        if gap > burst_gap_threshold_s and self.byte_buffer:
            res = self._flush_buffer()
            if res:
                out_frames.append(res)

        # Check if this new byte begins an ESC response or a new FC command
        if not self.byte_buffer:
            self.frame_start_time = current_start
            # If arriving 10µs..80µs after an FC frame, this burst is the ESC Telemetry response
            if self.is_bidirectional and self.last_fc_end_time is not None:
                fc_gap = float(current_start - self.last_fc_end_time)
                if (self.symbol_us * 6) / 1e6 <= fc_gap <= (self.symbol_us * 60) / 1e6:
                    self.current_burst_is_esc = True
                else:
                    self.current_burst_is_esc = False
            else:
                self.current_burst_is_esc = False

        self.byte_buffer.append((byte_val, current_start, current_end))
        self.last_byte_end_time = current_end

        # Process complete bursts
        if not self.current_burst_is_esc:
            # FC frame is exactly 2 bytes (16 bits)
            if len(self.byte_buffer) == 2:
                res = self._decode_fc_frame(
                    self.byte_buffer[0][0],
                    self.byte_buffer[1][0],
                    self.frame_start_time,
                    current_end,
                )
                self.last_fc_end_time = current_end
                self.byte_buffer.clear()
                if res:
                    out_frames.append(res)
        else:
            # ESC Telemetry frame is 3 bytes (20-21 bits GCR)
            if len(self.byte_buffer) >= 3:
                raw_gcr = ((self.byte_buffer[0][0] & 0xFF) << 16) | \
                          ((self.byte_buffer[1][0] & 0xFF) << 8) | \
                          (self.byte_buffer[2][0] & 0xFF)
                raw_20bit = (raw_gcr >> 4) & 0xFFFFF
                res = self._decode_esc_gcr(raw_20bit, self.frame_start_time, current_end)
                self.last_fc_end_time = None
                self.current_burst_is_esc = False
                self.byte_buffer.clear()
                if res:
                    out_frames.append(res)

        if len(out_frames) == 1:
            return out_frames[0]
        elif len(out_frames) > 1:
            return out_frames

        return None

    def _flush_buffer(self):
        """Flushes remaining bytes on an idle timeout."""
        if not self.byte_buffer:
            return None

        count = len(self.byte_buffer)
        start_t = self.frame_start_time
        end_t = self.byte_buffer[-1][2]

        if not getattr(self, "current_burst_is_esc", False) or count == 2:
            if count >= 2:
                b0 = self.byte_buffer[0][0]
                b1 = self.byte_buffer[1][0]
                self.byte_buffer.clear()
                self.last_fc_end_time = end_t
                return self._decode_fc_frame(b0, b1, start_t, end_t)
        else:
            # ESC Telemetry
            if count >= 3:
                raw_gcr = ((self.byte_buffer[0][0] & 0xFF) << 16) | \
                          ((self.byte_buffer[1][0] & 0xFF) << 8) | \
                          (self.byte_buffer[2][0] & 0xFF)
                raw_20bit = (raw_gcr >> 4) & 0xFFFFF
            elif count == 2:
                raw_gcr = ((self.byte_buffer[0][0] & 0xFF) << 8) | (self.byte_buffer[1][0] & 0xFF)
                raw_20bit = (raw_gcr << 4) & 0xFFFFF
            else:
                self.byte_buffer.clear()
                return None

            self.byte_buffer.clear()
            self.last_fc_end_time = None
            self.current_burst_is_esc = False
            return self._decode_esc_gcr(raw_20bit, start_t, end_t)

        self.byte_buffer.clear()
        return None
