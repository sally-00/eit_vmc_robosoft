"""Offline geometry checks; no robot connection or motion."""
import math
import sys
import types
import unittest
from unittest.mock import patch

import collect
from eit_reader import (EITReader, adjacent_protocol, measurement_protocol, parse_frame, supported_firmware,
                        gain_exponent, gain_rows_with_rx_margin)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = {
            "robot_ip": "192.0.2.1", "active_radius_m": .045,
            "edge_margin_m": .005, "grid_spacing_m": .01,
            "expected_tcp_offset_pose": [.09, 0, 0, math.pi / 2, 0, 0],
            "probe_axis_in_tcp": [0, 0, -1],
        }
        self.marks = {
            "east": [.045, 0, .1, 0, 0, 0],
            "west": [-.045, 0, .1, 0, 0, 0],
            "north": [0, .045, .1, 0, 0, 0],
            "south": [0, -.045, .1, 0, 0, 0],
            "contact": [0, 0, .08, 0, 0, 0],
        }

    def test_center_and_grid_keep_full_flat_inside_active_disk(self):
        cal = collect.calibration_from_marks(self.marks, self.cfg)
        self.assertEqual(cal["center_hover_base_m"][:2], [0, 0])
        self.assertEqual(collect.target_pose(cal, (.01, 0), .015)[:3].tolist(), [.01, 0, .095])
        self.assertEqual(len(collect.grid(self.cfg)), 37)
        for x, y in collect.grid(self.cfg):
            self.assertLessEqual(math.hypot(x, y) + .005 + self.cfg["edge_margin_m"],
                                 self.cfg["active_radius_m"] + 1e-9)

    def test_bad_center_and_fixture_pose_are_rejected(self):
        wrong = dict(self.marks)
        wrong["north"] = [.008, .045, .1, 0, 0, 0]
        with self.assertRaises(ValueError):
            collect.calibration_from_marks(wrong, self.cfg)
        with self.assertRaises(RuntimeError):
            collect.check_fixture_orientation([0, 0, 0, 0, .1, 0], self.cfg)
        wrong_cfg = {**self.cfg, "expected_tcp_offset_pose": [.09, 0, 0, 0, 0, 0]}
        with self.assertRaises(RuntimeError):
            collect.check_fixture_orientation(self.marks["contact"], wrong_cfg)


class AmodoProtocolTests(unittest.TestCase):
    def test_reset_waits_for_its_reply_after_delayed_stop_acknowledgement(self):
        reader = EITReader({"eit_electrode_pins": list(range(1, 9))})
        with patch.object(reader, "_write") as write, patch.object(reader, "_line", side_effect=[
                "OK: Stopped streaming.", "OK: Reset complete."]):
            self.assertEqual(reader._response("reset", expected_prefix="OK: Reset"), "OK: Reset complete.")
        write.assert_called_once_with("reset")

    def test_receiver_margin_preserves_tx_order_and_clamps_minimum(self):
        protocol = [(1, 2, 3, 4), (1, 2, 4, 5), (1, 2, 5, 6)]
        rows = ["1 2 3 4 32 8", "1 2 4 5 /2 /32", "1 2 5 6 1 /64"]
        self.assertEqual(gain_rows_with_rx_margin(rows, protocol, 1),
                         ["1 2 3 4 32 4", "1 2 4 5 /2 /64", "1 2 5 6 1 /64"])
        self.assertEqual(gain_rows_with_rx_margin(rows, protocol, 2)[0], "1 2 3 4 32 2")
        self.assertEqual(gain_rows_with_rx_margin(rows, protocol, 0), rows)
        for invalid in (-1, 15, True, .5, "1"):
            with self.assertRaisesRegex(ValueError, "0–14"):
                EITReader({"eit_electrode_pins": list(range(1, 9)), "eit_rx_gain_margin_steps": invalid})
        for token in ("0", "3", "/128", "512", "nan"):
            with self.assertRaises(ValueError):
                gain_exponent(token)
        with self.assertRaisesRegex(ValueError, "electrode order"):
            gain_rows_with_rx_margin(["2 1 3 4 32 8", *rows[1:]], protocol, 1)
        with self.assertRaisesRegex(ValueError, "entries"):
            gain_rows_with_rx_margin(rows[:-1], protocol, 1)

    def test_receiver_margin_rejects_incorrect_device_readback(self):
        reader = EITReader({"eit_electrode_pins": list(range(1, 9)), "eit_rx_gain_margin_steps": 1})
        reader.autogain_gains = [" ".join([*map(str, pins), "32", "8"]) for pins in reader.protocol]
        with patch.object(reader, "_load_gain_rows") as load, patch.object(
                reader, "_dump_gains", return_value=reader.autogain_gains):
            with self.assertRaisesRegex(RuntimeError, "did not retain"):
                reader._apply_rx_gain_margin()
        self.assertTrue(all(row.split()[-1] == "4" for row in load.call_args.args[0]))

    def test_combined_and_opposite_drive_patterns(self):
        for count in (8, 16):
            pins=list(range(1,count+1))
            adjacent=measurement_protocol(pins,"adjacent")
            opposite=measurement_protocol(pins,"opposite")
            combined=measurement_protocol(pins,"combined")
            self.assertEqual(len(opposite),count//2*(count-4))
            self.assertEqual(combined,adjacent+opposite)
            self.assertEqual(len(set(combined)),len(combined))
            self.assertTrue(all(len(set(row))==4 for row in combined))
            self.assertEqual(combined[:len(adjacent)],adjacent_protocol(pins))
        combined=measurement_protocol(list(range(1,17)),"combined")
        self.assertEqual(len(combined),304)
        self.assertEqual(combined[208],(1,9,2,3))
        self.assertEqual(combined[292],(8,16,1,2))
        self.assertEqual(sum(row[:2]==(8,16) for row in combined),12)
        with self.assertRaises(ValueError):
            measurement_protocol(list(range(1,17)),"unknown")

    def test_incomplete_frames_are_discarded_without_padding(self):
        reader=EITReader({"eit_electrode_pins":list(range(1,17)),"eit_frames_per_capture":2})
        reader.ser=types.SimpleNamespace(reset_input_buffer=lambda:None)
        complete=",".join(["0.001|0.1"]*208)
        incomplete=",".join(["0.001|0.1"]*150)
        with patch.object(reader,"_line",side_effect=["discard boundary",incomplete,complete,complete]):
            result=reader.capture()
        self.assertEqual(result["discarded_invalid_frames"],1)
        self.assertEqual(result["frame_count"],2)
        self.assertTrue(all(len(frame["magnitude_Vrms"])==208 for frame in result["frames"]))

    def test_supported_firmware_versions_are_explicit(self):
        for version in ("OK: Amodo EIT v1.1.1 (test)", "OK: Amodo EIT 1.1.2",
                        "OK: Amodo EIT v1.2.1 (Aug 4 2026 09:41:54)"):
            self.assertTrue(supported_firmware(version), version)
        for version in ("OK: Amodo EIT v1.2.10", "OK: Amodo EIT v1.2.2",
                        "OK: Amodo EIT v2.0.0", "OK: Other device v1.2.1"):
            self.assertFalse(supported_firmware(version), version)

    def test_eight_electrode_order_and_frame_parser(self):
        pins = [8, 2, 3, 4, 5, 6, 7, 1]
        rows = adjacent_protocol(pins)
        self.assertEqual(len(rows), 40)
        self.assertEqual(rows[0][:2], (8, 2))
        self.assertTrue(all(len(set(row)) == 4 for row in rows))
        raw = ",".join(["1.000e-3|0.2500C"] + ["2.000e-3|-0.1000"] * 39)
        frame = parse_frame(raw, len(rows))
        self.assertEqual(frame["magnitude_Vrms"][0], .001)
        self.assertEqual(frame["phase_rad"][1], -.1)
        self.assertEqual(sum(frame["clipped"]), 1)
        with self.assertRaises(ValueError):
            parse_frame("1.0|0.0", len(rows))

    def test_sixteen_electrode_protocol_and_bad_mappings(self):
        pins = list(range(1, 17))
        rows = adjacent_protocol(pins)
        self.assertEqual(len(rows), 208)
        self.assertEqual(len(set(rows)), 208)
        self.assertTrue(all(len(set(row)) == 4 for row in rows))
        self.assertTrue(any(row[:2] == (16, 1) for row in rows))
        for electrode in pins:
            self.assertEqual(sum(row[0] == electrode for row in rows), 13)
        frame = parse_frame(",".join(["0.001|0.1"] * 208), 208)
        self.assertEqual(len(frame["magnitude_Vrms"]), 208)
        for bad in (None, list(range(1, 10)), [1] * 16, list(range(16))):
            with self.assertRaises(ValueError):
                adjacent_protocol(bad)

    def test_serial_setup_and_fresh_frame_capture(self):
        class FakeSerial:
            def __init__(self, *args, **kwargs):
                self.out = bytearray()
                self.commands = []
                self.loading = False
                self.streaming = False
                self.load_count = 0
                self.configurations = []
                self.reloads = 0

            @property
            def in_waiting(self):
                return len(self.out)

            def write(self, data):
                command = data.decode().strip()
                self.commands.append(command)
                if command == "load":
                    self.loading = True
                    self.load_count = 0
                    self.configurations = []
                    self.reloads += 1
                elif self.loading and command:
                    self.load_count += 1
                    self.configurations.append(command)
                elif self.loading:
                    self.loading = False
                    self.out.extend(b"OK: Electrode configuration complete.\r\n")
                elif command == "reset":
                    self.out.extend(b"OK: Reset complete.\r\n")
                elif command == "version":
                    self.out.extend(b"OK: Amodo EIT v1.1.1 (test)\r\n")
                elif command in ("freqkhz 50", "setperiods 1", "mode phase"):
                    self.out.extend(b"OK: Configured.\r\n")
                elif command == "autogain":
                    gains = (("32", "8"), ("/2", "/64"), ("1", "/32"))
                    self.configurations = [" ".join([*row.split()[:4], *gains[i % 3]])
                                           for i, row in enumerate(self.configurations)]
                    self.out.extend(b"OK: Autogain complete.\r\n")
                elif command == "dump":
                    self.out.extend(("\r\n".join(self.configurations)+"\r\n").encode() +
                                    f"OK: Dumped all loaded electrode configurations ({self.load_count}).\r\n".encode())
                elif command == "start":
                    self.streaming = True

            def flush(self):
                pass

            def reset_input_buffer(self):
                self.out.clear()

            def read(self, size):
                if not self.out and self.streaming:
                    self.out.extend(("1.0e-3|0.1," * (self.load_count - 1) + "1.0e-3|0.1C\r\n").encode())
                result = bytes(self.out[:size])
                del self.out[:size]
                return result

            def close(self):
                pass

        fake = FakeSerial()
        serial_mod = types.ModuleType("serial")
        serial_mod.Serial = lambda *a, **kw: fake
        tools_mod = types.ModuleType("serial.tools")
        ports_mod = types.ModuleType("serial.tools.list_ports")
        ports_mod.comports = lambda: []
        with patch.dict(sys.modules, {"serial": serial_mod, "serial.tools": tools_mod,
                                      "serial.tools.list_ports": ports_mod}):
            cases = [(count, pattern, margin) for count, pattern in
                     ((8,"adjacent"),(16,"adjacent"),(8,"opposite"),(16,"opposite"),(8,"combined"),(16,"combined"))
                     for margin in (0, 1, 2)]
            for count, pattern, margin in cases:
                with self.subTest(electrodes=count,drive_pattern=pattern,rx_margin=margin):
                    fake = FakeSerial()
                    channels = len(measurement_protocol(list(range(1,count+1)),pattern))
                    cfg = {"eit_port": "/dev/fake", "eit_electrode_pins": list(range(1, count+1)),
                           "eit_frames_per_capture": 2,"eit_drive_pattern":pattern,
                           "eit_rx_gain_margin_steps": margin}
                    reader = EITReader(cfg)
                    reader.open()
                    result = reader.capture()
                    self.assertEqual(result["frame_count"], 2)
                    self.assertEqual(len(result["frames"][0]["magnitude_Vrms"]), channels)
                    self.assertTrue(result["frames"][0]["clipped"][-1])
                    self.assertEqual(fake.load_count, channels)
                    self.assertEqual(len(reader.metadata()["gain_dump"]), channels)
                    self.assertEqual(reader.metadata()["electrode_count"], count)
                    self.assertEqual(reader.metadata()["drive_pattern"],pattern)
                    self.assertEqual(fake.reloads, 2 if margin else 1)
                    self.assertEqual(fake.commands.count("autogain"), 1)
                    original = reader.metadata()["autogain_gain_dump"]
                    active = reader.metadata()["gain_dump"]
                    self.assertEqual(active, gain_rows_with_rx_margin(original, reader.protocol, margin))
                    self.assertTrue(all(a.split()[:5] == b.split()[:5] for a, b in zip(original, active)))
                    self.assertEqual(reader.metadata()["rx_gain_margin_steps"], margin)
                    if margin:
                        self.assertIn(2, reader.metadata()["rx_margin_clamped_channels"])
                    reader.close()


if __name__ == "__main__":
    unittest.main()
