"""Explicit-port USB capture and passive UART monitoring. No motion commands."""
import argparse
import json
import math
import re
import time
from dataclasses import asdict
from pathlib import Path

try:
    from .protocol import LineParser
except ImportError:
    from protocol import LineParser


class CaptureError(ValueError):
    pass


def rgb565be_to_rgb(raw, width, height):
    if width < 1 or height < 1 or width > 1280 or height > 1024 or len(raw) != width * height * 2:
        raise CaptureError("Invalid dimensions or RGB565 payload size")
    out = bytearray(width * height * 3)
    for pixel in range(width * height):
        value = (raw[2 * pixel] << 8) | raw[2 * pixel + 1]
        r, g, b = (value >> 11) & 31, (value >> 5) & 63, value & 31
        out[3 * pixel:3 * pixel + 3] = bytes(((r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)))
    return bytes(out)


def _read_exact(stream, count, deadline):
    result = bytearray()
    while len(result) < count:
        if time.monotonic() > deadline:
            raise CaptureError(f"Timeout: received {len(result)} of {count} bytes")
        result.extend(stream.read(count - len(result)))
    return bytes(result)


def read_capture(stream, timeout_s=15):
    deadline = time.monotonic() + timeout_s
    line = bytearray()
    skipped = 0
    while True:
        if time.monotonic() > deadline:
            raise CaptureError("Timeout waiting for RGB565BE frame header")
        byte = stream.read(1)
        if not byte:
            continue
        if byte != b"\n":
            line.extend(byte)
            if len(line) > 256:
                raise CaptureError("Serial line exceeds 256 bytes; check baud and firmware")
            continue
        if line.endswith(b"\r"):
            line = line[:-1]
        if line.startswith(b"ERROR "):
            raise CaptureError("Camera replied: " + line.decode("ascii", errors="replace"))
        match = re.fullmatch(rb"RGB565BE ([0-9]+) ([0-9]+) ([0-9]+)", line)
        if match:
            width, height, size = map(int, match.groups())
            if not 1 <= width <= 1280 or not 1 <= height <= 1024 or size != 2 * width * height:
                raise CaptureError("Invalid frame dimensions or byte count")
            raw = _read_exact(stream, size, deadline)
            if _read_exact(stream, 5, deadline) != b"\nEND\n":
                raise CaptureError("Missing frame terminator; frame discarded")
            return width, height, raw
        skipped += 1
        if skipped > 128:
            raise CaptureError("Too many non-frame lines")
        line.clear()


def open_explicit_port(port, baud):
    try:
        import serial
    except ImportError as exc:
        raise CaptureError("pyserial is optional and not installed. Install with: python -m pip install pyserial") from exc
    connection = serial.Serial(port=None, baudrate=baud, timeout=0.15, write_timeout=2)
    # Setting signals before open reduces unintended resets; USB adapters may
    # still assert them while opening. Motors must remain physically disabled.
    connection.dtr = False
    connection.rts = False
    connection.port = port
    connection.open()
    return connection


def main(argv=None):
    parser = argparse.ArgumentParser(description="Camera USB capture or passive packet monitor; never sends motion commands")
    sub = parser.add_subparsers(dest="mode", required=True)
    capture = sub.add_parser("capture")
    capture.add_argument("--port", required=True, help="Explicit camera USB COM port")
    capture.add_argument("--baud", type=int, default=460800)
    capture.add_argument("--output", type=Path, required=True, help="Output PNG; accompanying .rgb565 is also saved")
    capture.add_argument("--timeout", type=float, default=15)
    capture.add_argument("--settle-seconds", type=float, default=2, help="Wait after opening USB before CAPTURE (some adapters reset the board)")
    monitor = sub.add_parser("monitor")
    monitor.add_argument("--port", required=True, help="Explicit passive USB-UART receive port")
    monitor.add_argument("--baud", type=int, default=115200)
    monitor.add_argument("--seconds", type=float, default=30)
    monitor.add_argument("--output", type=Path, help="JSON lines including parser error count")
    args = parser.parse_args(argv)
    connection = None
    try:
        duration = args.timeout if args.mode == "capture" else args.seconds
        if args.baud <= 0 or not math.isfinite(duration) or duration <= 0:
            raise CaptureError("Baud and duration must be positive")
        if args.mode == "capture" and (not math.isfinite(args.settle_seconds) or not 0 <= args.settle_seconds <= 30):
            raise CaptureError("settle-seconds must be finite and between 0 and 30")
        if args.mode == "capture":
            from PIL import Image
        connection = open_explicit_port(args.port, args.baud)
        if args.mode == "capture":
            time.sleep(args.settle_seconds)
            connection.reset_input_buffer()
            connection.write(b"CAPTURE\n")
            connection.flush()
            width, height, raw = read_capture(connection, args.timeout)
            rgb = rgb565be_to_rgb(raw, width, height)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            Image.frombytes("RGB", (width, height), rgb).save(args.output, format="PNG")
            args.output.with_suffix(".rgb565").write_bytes(raw)
            args.output.with_suffix(".json").write_text(json.dumps({"format": "RGB565BE", "width": width, "height": height,
                "provenance": "camera_capture_unclassified", "calibration_confirmed": False}, indent=2) + "\n", encoding="utf-8")
            print(f"Captured {width}x{height}: {args.output}")
        else:
            decoder = LineParser()
            deadline = time.monotonic() + args.seconds
            stream = None
            try:
                if args.output:
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    stream = args.output.open("w", encoding="utf-8")
                while time.monotonic() < deadline:
                    data = connection.read(512)
                    for packet in decoder.feed(data):
                        row = {"host_monotonic_s": time.monotonic(), "packet": asdict(packet), "parser_errors": decoder.errors}
                        rendered = json.dumps(row)
                        print(rendered)
                        if stream:
                            stream.write(rendered + "\n")
                rendered = json.dumps({"event": "monitor_finished", "parser_errors": decoder.errors})
                print(rendered)
                if stream:
                    stream.write(rendered + "\n")
            finally:
                if stream:
                    stream.close()
    except (ValueError, OSError, ImportError) as exc:
        parser.exit(2, f"Serial tool failed: {exc}\n")
    finally:
        if connection:
            connection.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
