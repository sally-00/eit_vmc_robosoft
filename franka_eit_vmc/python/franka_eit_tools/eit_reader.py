"""Amodo ARIA EITSYS USB serial reader for an 8- or 16-electrode pad.

Protocol follows the bundled v1.1.1 source and the connected v1.2.1 command help.
The device streams one CSV line per configured frame. Host timestamps mark line
receipt because the firmware's stream does not include device timestamps.
"""
from __future__ import annotations

import math
import re
import time
from datetime import datetime, timezone


def supported_firmware(response):
    """Keep an explicit version allowlist; do not accept unverified future minors."""
    return bool(re.search(r"\bAmodo EIT v?(?:1\.1\.\d+|1\.2\.1)(?=\s|\(|$)", response))


def measurement_protocol(pins, drive_pattern="adjacent"):
    """Adjacent voltage sensing with adjacent, opposite or combined current drives."""
    if not isinstance(pins, list) or len(pins) not in (8, 16) or any(type(p) is not int or not 1 <= p <= 64 for p in pins):
        raise ValueError("eit_electrode_pins must list 8 or 16 device pins in physical order around the pad (pins in 1–64)")
    count = len(pins)
    if len(set(pins)) != count:
        raise ValueError("eit_electrode_pins must be distinct")
    if drive_pattern not in ("adjacent", "opposite", "combined"):
        raise ValueError("eit_drive_pattern must be adjacent, opposite or combined")
    drives = []
    if drive_pattern in ("adjacent", "combined"):
        drives.extend((i, (i+1) % count) for i in range(count))
    if drive_pattern in ("opposite", "combined"):
        # Reversing an opposite pair does not create another independent drive.
        drives.extend((i, i+count//2) for i in range(count//2))
    rows = []
    for tx in drives:
        for sense in range(count):
            rx = (sense, (sense + 1) % count)
            if set(tx).isdisjoint(rx):
                rows.append((pins[tx[0]], pins[tx[1]], pins[rx[0]], pins[rx[1]]))
    return rows


def adjacent_protocol(pins):
    """Backward-compatible adjacent-only protocol with its original channel order."""
    return measurement_protocol(pins, "adjacent")


def gain_exponent(token, maximum=8):
    """Decode Amodo power-of-two gain tokens, e.g. 8 or /16."""
    if not re.fullmatch(r"/?[1-9][0-9]*", token):
        raise ValueError(f"Invalid Amodo gain: {token!r}")
    value = int(token.lstrip("/"))
    if value & (value-1):
        raise ValueError(f"Amodo gain must be a power of two: {token!r}")
    exponent = (value.bit_length()-1) * (-1 if token.startswith("/") else 1)
    if not -6 <= exponent <= maximum:
        raise ValueError(f"Amodo gain out of range: {token!r}")
    return exponent


def validate_rx_margin(steps):
    if type(steps) is not int or not 0 <= steps <= 14:
        raise ValueError("eit_rx_gain_margin_steps must be an integer in 0–14")
    return steps


def gain_rows_with_rx_margin(rows, protocol, steps):
    """Validate the complete dump and lower RX only, clamped at 1/64."""
    validate_rx_margin(steps)
    if len(rows) != len(protocol):
        raise ValueError(f"Amodo gain dump has {len(rows)} entries; expected {len(protocol)}")
    result = []
    for index, (row, expected) in enumerate(zip(rows, protocol), 1):
        fields = row.split()
        if len(fields) != 6:
            raise ValueError(f"Malformed Amodo gain dump at channel {index}")
        try:
            pins = tuple(map(int, fields[:4]))
        except ValueError as error:
            raise ValueError(f"Invalid Amodo gain dump pins at channel {index}") from error
        if pins != tuple(expected):
            raise ValueError(f"Amodo gain dump electrode order differs at channel {index}")
        tx = gain_exponent(fields[4], maximum=5)
        rx = max(-6, gain_exponent(fields[5])-steps)
        tokens = [str(2**exponent) if exponent >= 0 else f"/{2**(-exponent)}" for exponent in (tx, rx)]
        result.append(" ".join([*map(str, pins), *tokens]))
    return result


def parse_frame(line, expected=40):
    """Preserve transmitted fields and per-channel clipping in phase mode."""
    raw = line.decode("ascii", errors="strict").strip() if isinstance(line, bytes) else line.strip()
    fields = raw.split(",")
    if len(fields) != expected:
        raise ValueError(f"EIT frame has {len(fields)} fields; expected {expected}")
    magnitudes, phases, clipped = [], [], []
    for value in fields:
        flag = value.endswith("C")
        field = value[:-1] if flag else value
        parts = field.split("|")
        if len(parts) != 2:
            raise ValueError(f"Expected magnitude|phase field: {value!r}")
        magnitude, phase = map(float, parts)
        if not math.isfinite(magnitude) or magnitude < 0 or not math.isfinite(phase):
            raise ValueError("EIT frame has a non-finite or negative measurement")
        magnitudes.append(magnitude)
        phases.append(phase)
        clipped.append(flag)
    return {"raw_line": raw, "magnitude_Vrms": magnitudes,
            "phase_rad": phases, "clipped": clipped}


class EITReader:
    def __init__(self, cfg):
        self.cfg = cfg
        self.drive_pattern = cfg.get("eit_drive_pattern", "adjacent")
        self.protocol = measurement_protocol(cfg.get("eit_electrode_pins"), self.drive_pattern)
        self.port = cfg.get("eit_port")
        self.frequency = cfg.get("eit_frequency_khz", 50)
        self.periods = cfg.get("eit_periods_per_measurement", 1)
        self.frame_count = cfg.get("eit_frames_per_capture", 5)
        self.timeout = cfg.get("eit_capture_timeout_s", 10)
        self.rx_gain_margin_steps = validate_rx_margin(cfg.get("eit_rx_gain_margin_steps", 0))
        for name, value, maximum in (("eit_frequency_khz", self.frequency, 1000),
                                     ("eit_periods_per_measurement", self.periods, 10000),
                                     ("eit_frames_per_capture", self.frame_count, 1000)):
            if type(value) is not int or not 1 <= value <= maximum:
                raise ValueError(f"{name} must be an integer in 1–{maximum}")
        if not isinstance(self.timeout, (int, float)) or not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("eit_capture_timeout_s must be positive")
        self.ser = None
        self.version = None
        self.gains = None
        self.autogain_gains = None
        self.rx_margin_clamped_channels = []
        self._rx_buffer = bytearray()

    def _line(self, deadline):
        while time.monotonic() < deadline:
            if b"\n" in self._rx_buffer:
                end = self._rx_buffer.index(b"\n")
                line = bytes(self._rx_buffer[:end])
                del self._rx_buffer[:end + 1]
                if line.strip():
                    return line.decode("ascii", errors="replace").strip()
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            self._rx_buffer.extend(chunk)
            if len(self._rx_buffer) > 100000:
                raise RuntimeError("Amodo EIT serial line exceeds 100 kB")
        raise TimeoutError("Timed out waiting for Amodo EIT serial response")

    def _write(self, command):
        self.ser.write((command + "\r").encode("ascii"))
        self.ser.flush()

    def _response(self, command, timeout=3, expected_prefix=None):
        self._write(command)
        deadline = time.monotonic() + timeout
        while True:
            line = self._line(deadline)
            if line.startswith("ERROR:"):
                raise RuntimeError(f"Amodo EIT {command}: {line}")
            if line.startswith("OK:") and (expected_prefix is None or line.startswith(expected_prefix)):
                return line

    def _load_gain_rows(self, rows):
        self._write("load")
        for row in rows:
            self._write(row)
        result = self._response("")
        if result != "OK: Electrode configuration complete.":
            raise RuntimeError(f"Unexpected electrode load response: {result}")

    def _dump_gains(self):
        self._write("dump")
        rows = []
        deadline = time.monotonic() + 5
        while True:
            line = self._line(deadline)
            if line.startswith("ERROR:"):
                raise RuntimeError(f"Amodo gain dump: {line}")
            if line.startswith("OK:"):
                break
            rows.append(line)
        return gain_rows_with_rx_margin(rows, self.protocol, 0)

    def _apply_rx_gain_margin(self):
        """Device must be stopped. Always derive gains from the original autogain dump."""
        expected = gain_rows_with_rx_margin(self.autogain_gains, self.protocol, self.rx_gain_margin_steps)
        self._load_gain_rows(expected)
        self.gains = self._dump_gains()
        if self.gains != expected:
            raise RuntimeError("EIT did not retain the requested RX gain margin; streaming was not started")
        self.rx_margin_clamped_channels = [i+1 for i, row in enumerate(self.autogain_gains)
                                          if gain_exponent(row.split()[5])-self.rx_gain_margin_steps < -6]

    def open(self):
        try:
            import serial
            from serial.tools import list_ports
        except ImportError as error:
            raise RuntimeError("Install pyserial from requirements.txt for Amodo EIT collection") from error
        port = self.port
        if port is None:
            ports = [p.device for p in list_ports.comports() if p.vid == 0x0483 and p.pid == 0x5740]
            if len(ports) != 1:
                raise RuntimeError(f"Expected one Amodo USB device (0483:5740); found {ports}. Set eit_port explicitly")
            port = ports[0]
        self.port = port
        self.ser = serial.Serial(port, 115200, timeout=0.1, exclusive=True)
        try:
            # The supplied wrapper uses blank, STOP, RESET to leave an unknown state.
            for command in ("", "", "stop"):
                self._write(command)
            self.ser.reset_input_buffer()
            self._rx_buffer.clear()
            result = self._response("reset", expected_prefix="OK: Reset")
            if not result.startswith("OK: Reset"):
                raise RuntimeError(f"Unexpected reset response: {result}")
            self.version = self._response("version")
            if not supported_firmware(self.version):
                raise RuntimeError(f"Verify firmware compatibility before collection: {self.version}")
            self._response("freqkhz " + str(self.frequency))
            self._response("setperiods " + str(self.periods))
            self._response("mode phase")
            self._load_gain_rows([f"{txp} {txn} {rxp} {rxn} 1 1" for txp, txn, rxp, rxn in self.protocol])
            self._write("autogain")
            deadline = time.monotonic() + 60
            while True:
                line = self._line(deadline)
                if line.startswith("ERROR:"):
                    raise RuntimeError(f"Amodo autogain: {line}")
                if line.startswith("OK: Autogain complete."):
                    break
            self.autogain_gains = self._dump_gains()
            self.gains = list(self.autogain_gains)
            if self.rx_gain_margin_steps:
                self._apply_rx_gain_margin()
            self._write("start")
        except BaseException:
            self.close()
            raise

    def metadata(self):
        return {"port": self.port, "version_response": self.version,
                "frequency_khz": self.frequency, "periods_per_measurement": self.periods,
                "electrode_pins_pad_order": self.cfg["eit_electrode_pins"],
                "electrode_count": len(self.cfg["eit_electrode_pins"]),
                "measurement_channels": len(self.protocol),
                "drive_pattern": self.drive_pattern,
                "configuration_order_txp_txn_rxp_rxn": self.protocol,
                "rx_gain_margin_steps": self.rx_gain_margin_steps,
                "autogain_gain_dump": self.autogain_gains,
                "rx_margin_clamped_channels": self.rx_margin_clamped_channels,
                "gain_dump": self.gains, "mode": "magnitude_Vrms_and_phase_rad",
                "frame_timestamp_source": "host receipt; firmware has no frame timestamp"}

    def capture(self):
        if self.ser is None:
            raise RuntimeError("Amodo EIT serial port is closed")
        # Discard queued frames acquired during robot movement or settling.
        self.ser.reset_input_buffer()
        self._rx_buffer.clear()
        deadline = time.monotonic() + self.timeout
        self._line(deadline)  # Discard the frame that may span the buffer reset.
        frames = []
        discarded = 0
        last_invalid = None
        while len(frames) < self.frame_count:
            try:
                line = self._line(deadline)
            except TimeoutError as exc:
                raise TimeoutError(f"Received {len(frames)}/{self.frame_count} complete EIT frames; "
                                   f"discarded {discarded} invalid frames. "
                                   f"{last_invalid or 'Check USB connection and close other serial readers.'}") from exc
            if line.startswith("ERROR:"):
                raise RuntimeError(f"Amodo EIT stream: {line}")
            try:
                frame = parse_frame(line, len(self.protocol))
            except ValueError as exc:
                discarded += 1
                last_invalid = str(exc)
                continue
            frame["host_utc"] = datetime.now(timezone.utc).isoformat()
            frame["host_monotonic_s"] = time.monotonic()
            frames.append(frame)
        return {"frames": frames, "frame_count": len(frames), "discarded_invalid_frames": discarded}

    def close(self):
        if self.ser is not None:
            ser = self.ser
            try:
                try:
                    self._write("stop")
                except Exception:
                    # A disconnected device cannot accept STOP, but the local
                    # descriptor and its exclusive lock still need releasing.
                    pass
            finally:
                try:
                    ser.close()
                finally:
                    self.ser = None
                    self._rx_buffer.clear()
