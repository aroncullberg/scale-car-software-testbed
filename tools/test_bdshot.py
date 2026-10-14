#!/usr/bin/env python3
"""Unit tests for Bidirectional DShot HLA decoder."""

import sys
from pathlib import Path

# Add tools directory to path
sys.path.insert(0, str(Path(__file__).parent / "saleae_bidirectional_dshot"))

import BidirectionalDshot


class MockFrame:
    def __init__(self, data_byte, start_t, end_t):
        self.type = "data"
        self.data = {"data": bytes([data_byte])}
        self.start_time = start_t
        self.end_time = end_t


def test_fc_throttle_decode():
    hla = BidirectionalDshot.Hla()
    hla.mode = "Bidirectional (Inverted / Idle HIGH)"
    hla.is_bidirectional = True

    # Test throttle = 1000 (0x03E8), telem_req = 0 -> Word = 0x7D05
    # Transmitted inverted: b0 = 0x82, b1 = 0xFA
    frame = hla._decode_fc_frame(0x82, 0xFA, 0.0, 1.0)
    assert frame is not None
    assert frame.type == "fc_throttle"
    assert frame.data["throttle"] == 1000
    assert frame.data["crc_ok"] is True
    print("✓ FC Throttle frame test passed!")


def test_esc_gcr_decode():
    hla = BidirectionalDshot.Hla()
    hla.motor_poles = 14
    hla.poles = 14

    raw_20bit = (0x1B << 15) | (0x12 << 10) | (0x13 << 5) | 0x0E

    frame = hla._decode_esc_gcr(raw_20bit, 0.0, 1.0)
    assert frame is not None
    assert frame.type == "esc_erpm"
    assert frame.data["crc_status"] == "OK"
    assert frame.data["period_us"] == 291
    assert frame.data["erpm"] == 206185
    assert frame.data["rpm"] == 29455
    print(f"✓ ESC GCR eRPM decode test passed: {frame.data['rpm']} RPM, eRPM={frame.data['erpm']}")


def test_end_to_end_stream():
    hla = BidirectionalDshot.Hla()
    hla.mode = "Bidirectional (Inverted / Idle HIGH)"
    hla.is_bidirectional = True
    hla.motor_poles = 14
    hla.poles = 14

    # 1. FC packet (2 bytes): Throttle 1000
    f1 = hla.decode(MockFrame(0x82, 0.000000, 0.000013))
    assert f1 is None  # waiting for 2nd byte
    f2 = hla.decode(MockFrame(0xFA, 0.000013, 0.000026))
    assert f2 is not None
    assert f2.type == "fc_throttle"
    assert f2.data["throttle"] == 1000

    # 2. Turnaround gap = 30us
    # 3. ESC Telemetry packet (3 bytes)
    # raw_20bit = 0xD926E => 3 bytes = [0xD9, 0x26, 0xE0]
    esc_gcr_20 = (0x1B << 15) | (0x12 << 10) | (0x13 << 5) | 0x0E
    b0 = (esc_gcr_20 >> 12) & 0xFF
    b1 = (esc_gcr_20 >> 4) & 0xFF
    b2 = (esc_gcr_20 << 4) & 0xFF

    e1 = hla.decode(MockFrame(b0, 0.000056, 0.000067))
    assert e1 is None
    e2 = hla.decode(MockFrame(b1, 0.000067, 0.000078))
    assert e2 is None
    e3 = hla.decode(MockFrame(b2, 0.000078, 0.000089))
    assert e3 is not None
    assert e3.type == "esc_erpm"
    assert e3.data["rpm"] == 29455

    # 4. Next FC frame at t=0.001000s (1ms later)
    n1 = hla.decode(MockFrame(0x82, 0.001000, 0.001013))
    assert n1 is None
    n2 = hla.decode(MockFrame(0xFA, 0.001013, 0.001026))
    assert n2 is not None
    assert n2.type == "fc_throttle"
    assert n2.data["throttle"] == 1000

    print("✓ Full stream simulation test (FC -> 30µs -> ESC -> 1ms -> FC) passed perfectly!")


if __name__ == "__main__":
    test_fc_throttle_decode()
    test_esc_gcr_decode()
    test_end_to_end_stream()
    print("All tests passed!")
