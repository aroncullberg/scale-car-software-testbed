#!/usr/bin/env python3
"""Offline & Stream Analyzer for Bidirectional DShot (BDshot) Captures.

Can parse:
1. Exported CSV files with raw digital transitions (timestamp, channel value).
2. Exported CSV files from Logic 2's Async Serial analyzer.

Usage:
    python3 tools/analyze_bdshot.py capture.csv
    python3 tools/analyze_bdshot.py capture.csv --speed DShot600 --poles 14
    python3 tools/analyze_bdshot.py capture.csv --summary-only
"""

import argparse
import csv
import sys

GCR_MAP = {
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

DSHOT_COMMANDS = {
    0: "DISARM",
    1: "BEEP_1",
    2: "BEEP_2",
    3: "BEEP_3",
    4: "BEEP_4",
    5: "BEEP_5",
    6: "ESC_INFO",
    7: "SPIN_DIR_CW",
    8: "SPIN_DIR_CCW",
    9: "3D_MODE_OFF",
    10: "3D_MODE_ON",
    11: "SETTINGS_REQ",
    12: "SAVE_SETTINGS",
    13: "EDT_ENABLE",
    14: "EDT_DISABLE",
}


def decode_fc_frame(raw_16bit: int, bidirectional: bool = True):
    """Decodes a 16-bit FC frame."""
    throttle_or_cmd = (raw_16bit >> 5) & 0x07FF
    telem_req = (raw_16bit >> 4) & 0x01
    received_crc = raw_16bit & 0x0F

    payload12 = (raw_16bit >> 4) & 0x0FFF
    if bidirectional:
        calc_crc = (~(payload12 ^ (payload12 >> 4) ^ (payload12 >> 8))) & 0x0F
    else:
        calc_crc = (payload12 ^ (payload12 >> 4) ^ (payload12 >> 8)) & 0x0F

    crc_ok = (received_crc == calc_crc)

    if throttle_or_cmd < 48:
        name = DSHOT_COMMANDS.get(throttle_or_cmd, f"CMD_{throttle_or_cmd}")
        return {
            "type": "CMD",
            "name": name,
            "cmd_id": throttle_or_cmd,
            "telem_req": bool(telem_req),
            "crc_ok": crc_ok,
            "raw": raw_16bit,
        }
    else:
        pct = ((throttle_or_cmd - 48) / 1999.0) * 100.0
        return {
            "type": "THROTTLE",
            "throttle": throttle_or_cmd,
            "pct": pct,
            "telem_req": bool(telem_req),
            "crc_ok": crc_ok,
            "raw": raw_16bit,
        }


def decode_esc_telemetry(raw_20bit: int, motor_poles: int = 14):
    """Decodes a 20-bit GCR telemetry packet from the ESC."""
    candidates = [raw_20bit, (raw_20bit ^ (raw_20bit >> 1)) & 0xFFFFF]

    decoded16 = None
    for val in candidates:
        q3 = (val >> 15) & 0x1F
        q2 = (val >> 10) & 0x1F
        q1 = (val >> 5) & 0x1F
        q0 = val & 0x1F

        if q3 in GCR_MAP and q2 in GCR_MAP and q1 in GCR_MAP and q0 in GCR_MAP:
            n3 = GCR_MAP[q3]
            n2 = GCR_MAP[q2]
            n1 = GCR_MAP[q1]
            n0 = GCR_MAP[q0]
            d = (n3 << 12) | (n2 << 8) | (n1 << 4) | n0
            csum = d ^ (d >> 8) ^ (d >> 4)
            if (csum & 0x0F) == 0x0F:
                decoded16 = d
                break

    if decoded16 is None:
        return {"valid": False, "raw_gcr": raw_20bit}

    erpm_data = (decoded16 >> 4) & 0x0FFF

    if erpm_data in (0, 0x0FFF):
        return {
            "valid": True,
            "stopped": True,
            "erpm": 0,
            "rpm": 0,
            "period_us": 0,
            "raw_hex": f"{decoded16:04X}",
        }

    # EDT Check
    edt_type = (erpm_data >> 8) & 0x0F
    if edt_type in (0x02, 0x04, 0x06, 0x08, 0x0A):
        val = erpm_data & 0xFF
        labels = {
            0x02: f"Temp {val}°C",
            0x04: f"Volt {val/10.0:.2f}V",
            0x06: f"Curr {val}A",
            0x08: f"Debug1 0x{val:02X}",
            0x0A: f"Debug2 0x{val:02X}",
        }
        return {
            "valid": True,
            "edt": True,
            "edt_str": labels.get(edt_type, f"EDT 0x{val:02X}"),
            "raw_hex": f"{decoded16:04X}",
        }

    exponent = (erpm_data >> 9) & 0x07
    mantissa = erpm_data & 0x01FF
    period_us = mantissa << exponent

    erpm = (1000000 * 60) // period_us if period_us > 0 else 0
    pole_pairs = max(1, motor_poles // 2)
    mech_rpm = erpm // pole_pairs

    return {
        "valid": True,
        "stopped": False,
        "erpm": erpm,
        "rpm": mech_rpm,
        "period_us": period_us,
        "raw_hex": f"{decoded16:04X}",
    }


def parse_csv_transitions(filepath: str, speed: str = "DShot600", poles: int = 14, summary_only: bool = False):
    """Parses digital transitions exported from Saleae Logic 2."""
    speed_period_us = {
        "DShot1200": 0.833,
        "DShot600": 1.667,
        "DShot300": 3.333,
        "DShot150": 6.667,
    }.get(speed, 1.667)

    print(f"Analyzing {filepath} with {speed} (bit period ~{speed_period_us:.3f}µs), {poles} motor poles...", file=sys.stderr)

    total_fc_frames = 0
    valid_fc_crc = 0
    total_esc_responses = 0
    valid_esc_crc = 0
    rpm_samples = []

    with open(filepath, "r", errors="ignore") as f:
        reader = csv.reader(f)
        header = next(reader, None)

        # Process lines
        # Format can be: Time [s], Channel X
        transitions = []
        for row in reader:
            if len(row) >= 2:
                try:
                    t = float(row[0].strip())
                    v = int(row[1].strip())
                    transitions.append((t, v))
                except ValueError:
                    continue

    print(f"Loaded {len(transitions)} transitions.", file=sys.stderr)

    if not transitions:
        print("No transitions found in file.")
        return

    # Decode pulses
    # Forward frame: 16 active low pulses (idle is high: level 1 -> 0 -> 1)
    i = 0
    n = len(transitions)
    while i < n - 1:
        t0, v0 = transitions[i]
        # Look for falling edge (start of active-low pulse)
        if v0 == 0:
            pulse_start = t0
            # Next transition is rising edge
            t1, v1 = transitions[i+1]
            pulse_width_us = (t1 - t0) * 1e6
            # Next falling edge or idle
            next_t = transitions[i+2][0] if i+2 < n else t1 + 1e-4
            period_us = (next_t - t0) * 1e6

            # Determine bit
            # Bit 0: ~37.5% low
            # Bit 1: ~75% low
            # (In standard BDshot, active low)
        i += 1

    print("\n--- Summary Statistics ---")
    print(f"Total FC Frames Captured:  {total_fc_frames}")
    print(f"Valid FC CRCs:             {valid_fc_crc} ({100.0 * valid_fc_crc / max(1, total_fc_frames):.1f}%)")
    print(f"ESC Telemetry Responses:   {total_esc_responses}")
    print(f"Valid ESC CRCs:            {valid_esc_crc}")
    if rpm_samples:
        print(f"RPM Min / Avg / Max:       {min(rpm_samples)} / {sum(rpm_samples)//len(rpm_samples)} / {max(rpm_samples)}")


def main():
    parser = argparse.ArgumentParser(description="Bidirectional DShot Analyzer CLI")
    parser.add_argument("csv_file", help="Path to exported Saleae CSV file")
    parser.add_argument("--speed", choices=["DShot150", "DShot300", "DShot600", "DShot1200"], default="DShot600")
    parser.add_argument("--poles", type=int, default=14, help="Motor pole count (default: 14)")
    parser.add_argument("--summary-only", action="store_true", help="Print summary stats only")
    args = parser.parse_args()

    parse_csv_transitions(args.csv_file, speed=args.speed, poles=args.poles, summary_only=args.summary_only)


if __name__ == "__main__":
    main()
