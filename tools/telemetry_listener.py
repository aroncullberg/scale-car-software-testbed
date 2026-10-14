#!/usr/bin/env python3
"""Connect to the car's blackboard TCP telemetry stream and decode records.

Join the car's WiFi (SoftAP "car-telemetry"), then:

    python3 tools/telemetry_listener.py            # print records, save raw to telemetry.bin
    python3 tools/telemetry_listener.py --host 192.168.4.1 --port 3333 --out run.bin

Record wire format (see docs/blackboard.md):
    offset 0  : topic_id        (1 byte)
    offset 1  : payload_len     (1 byte)
    offset 2  : timestamp_us    (6 bytes, little-endian, 48-bit)
    offset 8  : payload          (payload_len bytes, raw producer struct)

Payloads are raw C++ struct bytes including alignment padding, so this tool
prints topic / timestamp / hex by default rather than guessing field offsets.
The raw capture file holds the exact stream for offline decoding.
"""

import argparse
import socket
import sys

HEADER = 8

TOPICS = {
    1: "battery",
    2: "gps",
    3: "euler",
    4: "link_stats",
    5: "rc_channels",
    6: "rpm",
    7: "flight_mode",
    8: "accel",
    9: "gyro",
    10: "quat",
}


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--port", type=int, default=3333)
    ap.add_argument("--out", default="telemetry.bin")
    ap.add_argument("--quiet", action="store_true", help="only save, do not print")
    args = ap.parse_args()

    print(f"connecting to {args.host}:{args.port} ...", file=sys.stderr)
    sock = socket.create_connection((args.host, args.port))
    print("connected", file=sys.stderr)

    count = 0
    with open(args.out, "wb") as raw:
        while True:
            header = recv_exact(sock, HEADER)
            if header is None:
                print("\nstream closed", file=sys.stderr)
                break
            topic = header[0]
            payload_len = header[1]
            ts = int.from_bytes(header[2:8], "little")
            payload = recv_exact(sock, payload_len) if payload_len else b""
            if payload is None:
                print("\nstream closed mid-record", file=sys.stderr)
                break

            raw.write(header)
            raw.write(payload)
            count += 1

            if not args.quiet:
                name = TOPICS.get(topic, f"topic{topic}")
                print(f"{ts/1e6:12.3f}s  {name:<12} {payload.hex()}")


if __name__ == "__main__":
    main()
