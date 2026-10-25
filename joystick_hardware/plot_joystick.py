#!/usr/bin/env python3
"""Save timestamped ESP32 joystick samples and optionally plot them live."""

import argparse
import csv
from collections import deque
from datetime import datetime
from pathlib import Path
import sys
import time


def parse_sample(line):
    """Return (ESP milliseconds, X, Y, pressed), or None for a header/bad line."""
    parts = line.strip().split(",")
    if len(parts) != 4:
        return None
    try:
        values = tuple(int(part) for part in parts)
    except ValueError:
        return None
    esp_ms, x_raw, y_raw, pressed = values
    if esp_ms < 0 or not (0 <= x_raw <= 4095 and 0 <= y_raw <= 4095) or pressed not in (0, 1):
        return None
    return values


def choose_port(requested):
    if requested:
        return requested
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    esp32_ports = [port for port in ports if (port.vid, port.pid) == (0x1A86, 0x7523)]
    if len(esp32_ports) == 1:
        return esp32_ports[0].device
    found = ", ".join(port.device for port in ports) or "none"
    raise SystemExit(f"Expected one ESP32 CH340 serial port (1a86:7523); found {len(esp32_ports)}. Detected serial ports: {found}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="ESP32 serial port; auto-detects the CH340 adapter by USB ID when omitted")
    parser.add_argument("--detect-port", action="store_true", help="Print the detected ESP32 serial port and exit")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", type=Path, help="CSV path; defaults to a new timestamped file")
    parser.add_argument("--no-plot", action="store_true", help="Save and print samples without a GUI")
    parser.add_argument("--window", type=float, default=15.0, help="Visible plot duration in seconds")
    args = parser.parse_args()
    if args.window <= 0:
        parser.error("--window must be positive")

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Install pyserial: python -m pip install pyserial") from exc

    port = choose_port(args.port)
    if args.detect_port:
        print(port)
        return

    plt = None
    if not args.no_plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise SystemExit("Install matplotlib or pass --no-plot") from exc

    output = args.output or Path("recordings") / f"joystick_{datetime.now().astimezone():%Y%m%d_%H%M%S}.csv"
    output.parent.mkdir(parents=True, exist_ok=True)

    with serial.Serial(port, args.baud, timeout=0.02) as device, output.open("x", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["host_unix_ns", "host_monotonic_ns", "esp_ms", "x_raw", "y_raw", "pressed"])
        file.flush()
        print(f"Recording {port} to {output.resolve()} (Ctrl+C to stop)", flush=True)

        if plt is not None:
            plt.ion()
            fig, ax = plt.subplots()
            (x_line,) = ax.plot([], [], label="VRx")
            (y_line,) = ax.plot([], [], label="VRy")
            ax.set(xlabel="Time since start (s)", ylabel="ESP32 ADC count", ylim=(-100, 4200))
            ax.legend(loc="upper right")
            fig.tight_layout()
            fig.show()
            samples = deque()
            start_ns = None
            last_draw = time.monotonic()

        count = 0
        last_print = time.monotonic()
        try:
            while plt is None or plt.fignum_exists(fig.number):
                raw = device.readline()
                if raw:
                    host_monotonic_ns = time.monotonic_ns()
                    host_unix_ns = time.time_ns()
                    sample = parse_sample(raw.decode("ascii", errors="replace"))
                    if sample is not None:
                        writer.writerow([host_unix_ns, host_monotonic_ns, *sample])
                        file.flush()
                        count += 1
                        if plt is not None:
                            if start_ns is None:
                                start_ns = host_monotonic_ns
                            t = (host_monotonic_ns - start_ns) / 1e9
                            samples.append((t, sample[1], sample[2]))
                            while samples and samples[0][0] < t - args.window:
                                samples.popleft()
                        elif time.monotonic() - last_print >= 0.5:
                            print(f"x={sample[1]:4d}  y={sample[2]:4d}  pressed={sample[3]}", flush=True)
                            last_print = time.monotonic()

                if plt is not None and time.monotonic() - last_draw >= 0.05:
                    if samples:
                        t0 = samples[-1][0]
                        x_line.set_data([s[0] for s in samples], [s[1] for s in samples])
                        y_line.set_data([s[0] for s in samples], [s[2] for s in samples])
                        ax.set_xlim(max(0, t0 - args.window), max(args.window, t0))
                    plt.pause(0.001)
                    last_draw = time.monotonic()
        except KeyboardInterrupt:
            pass
        finally:
            print(f"Saved {count} samples to {output.resolve()}", flush=True)


if __name__ == "__main__":
    main()
