"""Offline signal, model persistence and GUI workflow checks. No hardware access."""
import argparse
import json
import os
from pathlib import Path
import tempfile
import time
import threading
import unittest
from unittest.mock import patch

import numpy as np

from contact_position import (DemoReader, PositionModel, fit_samples,
                              plate_triangles, signature, targets, voltages)
from contact_signal_debug import SignalDebugPlot, signal_summary
from contact_mlp import ContactMLP
from eit_reader import measurement_protocol


class SignalDebugTests(unittest.TestCase):
    def capture(self, magnitude=0.01, phase=0., clipped=False, channels=40):
        return {"frames": [{"magnitude_Vrms": [magnitude] * channels,
                            "phase_rad": [phase] * channels, "clipped": [clipped] * channels}
                           for _ in range(5)]}

    def test_units_baseline_change_and_visible_clipping(self):
        baseline = self.capture()
        clipped = self.capture(magnitude=.011, clipped=True)
        summary = signal_summary(clipped, baseline)
        np.testing.assert_allclose(summary["magnitude_mV"], 11)
        np.testing.assert_allclose(summary["relative_percent"].real, 10)
        self.assertAlmostEqual(summary["rms_change_percent"], 10.)
        self.assertTrue(summary["clipped"].all())
        with self.assertRaises(ValueError):
            voltages(clipped)  # Viewing a clipped signal must not admit it to training.
        without_baseline = signal_summary(clipped)
        self.assertTrue(np.isnan(without_baseline["relative_percent"].real).all())
        self.assertTrue(np.isnan(without_baseline["relative_percent"].imag).all())

    def test_circular_phase_mean_and_missing_channels(self):
        capture = self.capture(phase=np.pi - .01)
        capture["frames"][0]["phase_rad"] = [-np.pi + .01] * 40
        phase = signal_summary(capture)["phase_rad"]
        self.assertTrue((np.abs(phase) > 3.).all())
        zero = self.capture(magnitude=0)
        self.assertTrue(signal_summary(zero)["zero_channels"].all())
        with self.assertRaises(ValueError):
            signal_summary({"frames": []})

    def test_plot_baseline_reset_old_preview_and_error(self):
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        figure = Figure()
        FigureCanvasAgg(figure)
        plot = SignalDebugPlot(figure)
        baseline = self.capture()
        plot.update(baseline, baseline, "Baseline", 10.)
        plot.update(self.capture(.011), baseline, "Preview", 11.)
        self.assertEqual(len(plot.history), 2)
        self.assertAlmostEqual(plot.summary["rms_change_percent"], 10.)
        plot.update(baseline, baseline, "Baseline", 12.)
        self.assertEqual(len(plot.history), 1)
        plot.update(self.capture(.02), baseline, "Old preview", 11.5)
        self.assertEqual(plot.last_sample_s, 12.)
        self.assertAlmostEqual(plot.summary["rms_change_percent"], 0.)
        plot.unavailable("USB disconnected")
        self.assertEqual(len(plot.magnitude.get_xdata()), 0)
        self.assertIn("USB disconnected", plot.description)
        large = self.capture(channels=208)
        plot.update(large, large, "16 electrode baseline", 13.)
        self.assertEqual(len(plot.magnitude.get_xdata()), 208)
        self.assertIn("208 measurement channels", plot.axes[0].get_title())
        self.assertIn("0/208", plot.description)
        with self.assertRaisesRegex(ValueError, "channel counts differ"):
            signal_summary(large, baseline)


class PositionTests(unittest.TestCase):
    electrode_count = 8
    drive_pattern = "adjacent"

    @classmethod
    def setUpClass(cls):
        cls.cfg = {"eit_frames_per_capture": 5, "eit_electrode_pins": list(range(1, cls.electrode_count+1)),
                   "eit_frequency_khz": 50, "eit_periods_per_measurement": 1,
                   "eit_drive_pattern": cls.drive_pattern}
        cls.metadata = {"radius_mm": 45., "numbering": "ccw", "demo": True, "eit_config": cls.cfg}
        cls.reader = DemoReader(cls.cfg)
        cls.plan = targets()
        cls.records = []
        for repeat in range(3):
            for i, xy in enumerate(cls.plan):
                cls.reader.xy = None
                baseline = cls.reader.capture()
                cls.reader.xy = xy
                cls.records.append({"point_id": i, "repeat": repeat, "xy_mm": xy.tolist(),
                                    "baseline": baseline, "contact": cls.reader.capture()})
        cls.model, cls.report = fit_samples(cls.records, dict(cls.metadata))

    def test_configurable_layout_inset_and_orientation_mesh(self):
        for count in (3, 20, 51):
            points = targets(count, 45, 5)
            self.assertEqual(points.shape, (count, 2))
            np.testing.assert_equal(points[0], [0, 0])
            self.assertLessEqual(np.linalg.norm(points, axis=1).max(), 40.000001)
        for count in (0, 2, 501):
            with self.assertRaises(ValueError):
                targets(count)
        for numbering in ("ccw", "cw"):
            mesh = plate_triangles(numbering)
            self.assertGreater(len(mesh), 1000)
            self.assertTrue(np.any(np.isclose(mesh[:, :, 2], 2.5)))

    def test_held_out_touches_and_unseen_synthetic_location(self):
        self.assertEqual(self.report["held_out_touches"], 20)
        self.assertLess(self.report["median_error_mm"], 2.)
        self.reader.xy = None
        baseline = self.reader.capture()
        self.reader.xy = [7., -9.]
        result = self.model.predict(self.reader.capture(), baseline)
        self.assertEqual(result["state"], "contact")
        self.assertLess(np.linalg.norm(np.array(result["xy_mm"]) - [7., -9.]), 6.)
        self.reader.xy = None
        self.assertNotEqual(self.model.predict(self.reader.capture(), baseline)["state"], "no_contact")

    def test_clipping_validation_and_prediction_for_novel_signal(self):
        self.reader.xy = None
        baseline = self.reader.capture()
        bad = json.loads(json.dumps(baseline))
        bad["frames"][0]["clipped"][0] = True
        with self.assertRaisesRegex(ValueError, "Clipped"):
            voltages(bad)
        novel = json.loads(json.dumps(baseline))
        for frame in novel["frames"]:
            frame["phase_rad"] = [1.] * len(frame["phase_rad"])
        result = self.model.predict(novel, baseline)
        self.assertEqual(result["state"], "unfamiliar")
        x, _ = signature(novel, baseline, channel_mask=self.model.channel_mask)
        expected, distance = self.model.estimate(x)
        np.testing.assert_allclose(result["xy_mm"], expected[0])
        self.assertTrue(np.isfinite(result["xy_mm"]).all())
        self.assertAlmostEqual(result["strength_rms_percent"], 100 * abs(np.exp(1j) - 1))
        self.assertAlmostEqual(result["pattern_distance"], distance[0])
        self.assertEqual(result["pattern_limit"], self.model.support_limit)
        self.assertFalse(result["pattern_familiar"])

    def test_strength_uses_usable_channels_and_familiarity_is_separate_from_pad_bounds(self):
        a = np.sqrt(.5)
        x = [[a, a, 0., 0., 0., 0.], [1., 0., 0., 0., 0., 0.], [-a, a, 0., 0., 0., 0.]]
        model = PositionModel(x, [[0., 0.], [10., 0.], [0., 10.]], 0.,
                              {"radius_mm": 45., "usable_channel_mask": [True, True, False]})
        def capture(values):
            return {"frames": [{"magnitude_Vrms": values, "phase_rad": [0.]*3,
                                "clipped": [False]*3} for _ in range(5)]}
        baseline = capture([.01, .01, 0.])
        touch = capture([.011, .011, 123.])
        result = model.predict(touch, baseline)
        self.assertAlmostEqual(result["strength_rms_percent"], 10.)
        self.assertAlmostEqual(result["strength"], np.sqrt(.02))
        self.assertAlmostEqual(result["pattern_distance"], 0., places=7)
        self.assertTrue(result["pattern_familiar"])
        self.assertTrue(result["within_pad"])
        with patch.object(model, "estimate", return_value=(np.array([[60., 0.]]), np.array([0.]))):
            outside = model.predict(touch, baseline)
        self.assertTrue(outside["pattern_familiar"])
        self.assertFalse(outside["within_pad"])
        self.assertEqual(outside["state"], "unfamiliar")
        self.assertEqual(outside["xy_mm"], [60., 0.])

    def test_zero_change_training_and_weak_prediction_ignore_legacy_threshold(self):
        zero_change = [{**r, "contact": r["baseline"]} for r in self.records[:3]]
        zero_model, report = fit_samples(zero_change, dict(self.metadata))
        self.assertEqual(zero_model.threshold, 0.)
        self.assertFalse(report["contact_strength_check"])
        result = zero_model.predict(zero_change[0]["contact"], zero_change[0]["baseline"])
        self.assertEqual(result["state"], "contact")
        self.assertEqual(result["strength"], 0.)

        # An old positive threshold must not suppress a tiny, familiar touch.
        record = self.records[0]
        weak = json.loads(json.dumps(record["contact"]))
        base = voltages(record["baseline"]).mean(axis=0)
        rows = base + 1e-4 * (voltages(record["contact"]) - base)
        for frame, row in zip(weak["frames"], rows):
            frame["magnitude_Vrms"] = np.abs(row).tolist()
            frame["phase_rad"] = np.angle(row).tolist()
        expected = self.model.predict(record["contact"], record["baseline"])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "legacy_threshold.npz"
            metadata = {**self.model.metadata, "model_version": 1}
            metadata.pop("contact_strength_check", None)
            np.savez_compressed(path, x=self.model.x, xy=self.model.xy, threshold=1e6,
                                metadata=json.dumps(metadata))
            loaded = PositionModel.load(path)
            result = loaded.predict(weak, record["baseline"])
            self.assertLess(result["strength"], loaded.threshold)
            self.assertEqual(result["state"], "contact")
            np.testing.assert_allclose(result["xy_mm"], expected["xy_mm"], atol=1e-6)
            zero_model.save(path)
            self.assertEqual(PositionModel.load(path).threshold, 0.)

    def test_model_round_trip_and_force_scale_invariance(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "model.npz"
            self.model.save(path)
            loaded = PositionModel.load(path)
            self.assertEqual(loaded.metadata["numbering"], "ccw")
            np.testing.assert_allclose(loaded.weights, self.model.weights)
            self.assertEqual(loaded.x.shape[1], 2*len(self.reader.protocol))
            self.assertEqual(loaded.metadata["drive_pattern"],self.drive_pattern)
        record = self.records[0]
        scaled = json.loads(json.dumps(record["contact"]))
        base = voltages(record["baseline"]).mean(axis=0)
        contact = voltages(record["contact"])
        shifted = base + .5 * (contact - base)
        for frame, row in zip(scaled["frames"], shifted):
            frame["magnitude_Vrms"] = np.abs(row).tolist()
            frame["phase_rad"] = np.angle(row).tolist()
        before, _ = signature(record["contact"], record["baseline"])
        after, _ = signature(scaled, record["baseline"])
        np.testing.assert_allclose(before, after, atol=1e-10)

    def test_incompatible_model_channel_count_is_rejected(self):
        other_count = 16 if self.electrode_count == 8 else 8
        other = DemoReader({"eit_electrode_pins": list(range(1, other_count+1))})
        baseline = other.capture()
        other.xy = [0, 0]
        with self.assertRaisesRegex(ValueError, "retrain"):
            self.model.predict(other.capture(), baseline)

    def test_allow_clipped_covers_training_prediction_and_saved_model(self):
        records = json.loads(json.dumps(self.records))
        for record in records:
            for kind in ("baseline", "contact"):
                for frame in record[kind]["frames"]:
                    frame["clipped"][0] = True
        with self.assertRaisesRegex(ValueError, "Clipped"):
            fit_samples(records, dict(self.metadata))
        sample = records[0]
        np.testing.assert_allclose(voltages(sample["contact"], allow_clipped=True),
                                   voltages(self.records[0]["contact"]))
        model, report = fit_samples(records, {**self.metadata, "allow_clipped": True})
        self.assertTrue(model.metadata["allow_clipped"])
        self.assertEqual(report["clipped_frames"], len(records)*10)
        self.assertEqual(model.predict(sample["contact"], sample["baseline"])["state"], "contact")
        with self.assertRaisesRegex(ValueError, "Clipped"):
            model.predict(sample["contact"], sample["baseline"], allow_clipped=False)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"allow_clipped.npz"
            model.save(path)
            loaded=PositionModel.load(path)
            self.assertTrue(loaded.metadata["allow_clipped"])
            self.assertEqual(loaded.predict(sample["contact"], sample["baseline"])["state"], "contact")
        with self.assertRaisesRegex(ValueError, "Empty"):
            voltages({"frames": []}, allow_clipped=True)
        invalid=json.loads(json.dumps(sample["contact"]))
        invalid["frames"][0]["magnitude_Vrms"][0]=0.
        self.assertTrue(np.isfinite(voltages(invalid, allow_clipped=True)).all())
        invalid["frames"][0]["magnitude_Vrms"][0]=-1.
        with self.assertRaisesRegex(ValueError, "Invalid EIT"):
            voltages(invalid, allow_clipped=True)
        self.assertTrue(sample["contact"]["frames"][0]["clipped"][0])

    def test_zero_baseline_channels_are_excluded_consistently(self):
        records=json.loads(json.dumps(self.records))
        for record in records:
            for kind in ("baseline", "contact"):
                for frame in record[kind]["frames"]:
                    frame["magnitude_Vrms"][0]=0.
        for frame in records[0]["baseline"]["frames"]:
            frame["magnitude_Vrms"][1]=0.
        model, report=fit_samples(records, dict(self.metadata))
        channels=len(self.reader.protocol)
        self.assertEqual(report["excluded_channels"], [1, 2])
        self.assertEqual(model.metadata["usable_measurement_channels"], channels-2)
        self.assertFalse(model.channel_mask[0])
        self.assertFalse(model.channel_mask[1])
        np.testing.assert_equal(model.x[:, [0, 1, channels, channels+1]], 0.)
        sample=records[0]
        self.assertEqual(model.predict(sample["contact"], sample["baseline"])["state"], "contact")
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"masked_model.npz"
            model.save(path)
            restored=PositionModel.load(path)
            np.testing.assert_equal(restored.channel_mask, model.channel_mask)
            self.assertEqual(restored.predict(sample["contact"], sample["baseline"])["state"], "contact")
        lost_baseline=json.loads(json.dumps(sample["baseline"]))
        for frame in lost_baseline["frames"]:
            frame["magnitude_Vrms"][2]=0.
        with self.assertRaisesRegex(ValueError, "required model channels"):
            model.predict(sample["contact"], lost_baseline)
        all_zero=json.loads(json.dumps(sample["baseline"]))
        for frame in all_zero["frames"]:
            frame["magnitude_Vrms"]=[0.]*channels
        with self.assertRaisesRegex(ValueError, "All EIT voltage channels are zero"):
            voltages(all_zero)


class SixteenPositionTests(PositionTests):
    electrode_count = 16


class CombinedPositionTests(SixteenPositionTests):
    drive_pattern = "combined"


class MLPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = {"eit_electrode_pins": list(range(1, 17)), "eit_drive_pattern": "combined",
                   "eit_frames_per_capture": 5}
        cls.metadata = {"radius_mm": 45., "numbering": "ccw", "demo": True,
                        "eit_config": cls.cfg, "model_type": "mlp"}
        cls.reader = DemoReader(cls.cfg)
        cls.records = []
        with patch("time.sleep"):
            for repeat in range(3):
                for point, xy in enumerate(targets()):
                    cls.reader.xy = None
                    baseline = cls.reader.capture()
                    cls.reader.xy = xy
                    cls.records.append({"point_id": point, "repeat": repeat, "xy_mm": xy.tolist(),
                                        "baseline": baseline, "contact": cls.reader.capture()})
        cls.model, cls.report = fit_samples(cls.records, dict(cls.metadata))

    def test_learns_position_and_predicts_unseen_synthetic_touch(self):
        self.assertEqual(self.model.model_type, "mlp")
        self.assertEqual(self.report["model_type"], "mlp")
        self.assertEqual(self.report["mlp_training"]["layers"], [608, 128, 64, 2])
        self.assertEqual(self.report["held_out_touches"], 20)
        self.assertLess(self.report["median_error_mm"], 2.)
        with patch("time.sleep"):
            self.reader.xy = None
            baseline = self.reader.capture()
            self.reader.xy = [7., -9.]
            result = self.model.predict(self.reader.capture(), baseline)
        self.assertLess(np.linalg.norm(np.asarray(result["xy_mm"])-[7., -9.]), 6.)
        self.assertIn("strength_rms_percent", result)
        self.assertIn("pattern_distance", result)

    def test_round_trip_preserves_predictions_without_training_on_load(self):
        record = self.records[0]
        expected = self.model.predict(record["contact"], record["baseline"])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"mlp.npz"
            self.model.save(path)
            with patch.object(ContactMLP, "fit", side_effect=AssertionError("Load must not train")):
                loaded = PositionModel.load(path)
            self.assertEqual(loaded.model_type, "mlp")
            self.assertEqual(loaded.predict(record["contact"], record["baseline"]), expected)
            with np.load(path, allow_pickle=False) as saved:
                broken = {name: saved[name] for name in saved.files if name != "mlp_weight_0"}
            np.savez_compressed(path, **broken)
            with self.assertRaisesRegex(ValueError, "missing mlp_weight_0"):
                PositionModel.load(path)
            broken["mlp_weight_0"] = np.zeros((1, 2))
            np.savez_compressed(path, **broken)
            with self.assertRaisesRegex(ValueError, "Invalid saved MLP"):
                PositionModel.load(path)

    def test_backward_compatible_rbf_and_invalid_model_type(self):
        x, xy = self.model.x[:3], self.model.xy[:3]
        metadata = {**self.metadata, "model_type": "invalid"}
        with self.assertRaisesRegex(ValueError, "rbf or mlp"):
            PositionModel(x, xy, 0., metadata)
        metadata.pop("model_type")
        metadata["model_version"] = 1
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"legacy.npz"
            np.savez_compressed(path, x=x, xy=xy, threshold=0., metadata=json.dumps(metadata))
            restored = PositionModel.load(path)
            self.assertEqual(restored.model_type, "rbf")

    def test_backpropagation_matches_finite_differences(self):
        rng = np.random.default_rng(4)
        x = rng.normal(size=(3, 4))
        xy = rng.normal(size=(3, 2))
        with patch.object(ContactMLP, "max_epochs", 1):
            network = ContactMLP(4, 45.)
            network.fit(x, xy)
        _, derivatives = network.loss_gradient(x, xy)
        for parameter, derivative in zip(network.parameters, derivatives):
            for index in (0, parameter.size-1):
                original = parameter.flat[index]
                parameter.flat[index] = original + 1e-6
                plus, _ = network.loss_gradient(x, xy)
                parameter.flat[index] = original - 1e-6
                minus, _ = network.loss_gradient(x, xy)
                parameter.flat[index] = original
                self.assertAlmostEqual(derivative.flat[index], (plus-minus)/2e-6, places=6)


@unittest.skipUnless(os.environ.get("DISPLAY"), "GUI requires a display")
class WorkflowTests(unittest.TestCase):
    electrode_count = 8
    drive_pattern = "adjacent"
    force_clipped = False
    force_zero = False
    training_model = "rbf"

    def setUp(self):
        if self.force_clipped or self.force_zero:
            original_capture = DemoReader.capture
            def clipped_capture(reader):
                capture = original_capture(reader)
                for frame in capture["frames"]:
                    if self.force_clipped:
                        frame["clipped"][0] = True
                    if self.force_zero:
                        frame["magnitude_Vrms"][0] = 0.
                return capture
            patcher = patch.object(DemoReader, "capture", clipped_capture)
            patcher.start()
            self.addCleanup(patcher.stop)

    def test_complete_guided_training_live_release_and_reload(self):
        import tkinter as tk
        from contact_position_app import ContactApp
        with tempfile.TemporaryDirectory() as folder:
            args = argparse.Namespace(demo=True, points=5, repeats=2, numbering="ccw", radius_mm=45.,
                                      margin_mm=5., output=Path(folder), model=None,
                                      training_model=self.training_model)
            cfg = {"eit_frames_per_capture": 5, "eit_electrode_pins": list(range(1, self.electrode_count+1)),
                   "eit_frequency_khz": 50, "eit_periods_per_measurement": 1,
                   "eit_allow_clipped": self.force_clipped,"eit_drive_pattern":self.drive_pattern}
            root = tk.Tk()
            root.withdraw()
            try:
                app = ContactApp(root, args, cfg)
                def settle():
                    deadline = time.monotonic() + 20
                    while app.busy:
                        root.update()
                        if time.monotonic() > deadline:
                            self.fail("UI worker timed out")
                        time.sleep(.01)
                    root.update()
                app.connect()
                settle()
                self.assertIsNotNone(app.reader, app.status.get())
                channels=len(measurement_protocol(cfg["eit_electrode_pins"],self.drive_pattern))
                self.assertIn(f"{channels} measurements/frame", app.scan_summary.get())
                # Preview runs while waiting for a touch, without recording/advancing it.
                original_capture = app.reader.capture
                in_flight = 0
                overlap = []
                counter_lock = threading.Lock()
                def checked_capture():
                    nonlocal in_flight
                    with counter_lock:
                        in_flight += 1
                        overlap.append(in_flight)
                    try:
                        return original_capture()
                    finally:
                        with counter_lock:
                            in_flight -= 1
                # Pause and drain an existing preview before instrumenting the reader.
                app.debug_enabled.set(False)
                deadline = time.monotonic() + 5
                while app.debug_busy and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.01)
                app.reader.capture = checked_capture
                app.debug_enabled.set(True)
                app.start_training()
                app.reader.xy = [0., 0.]
                deadline = time.monotonic() + 5
                while (app.signal_plot.summary is None or not np.isfinite(app.signal_plot.summary["magnitude_mV"]).all()
                       or not app.signal_plot.description.startswith("Live preview")) and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.01)
                self.assertIn("Live preview", app.signal_plot.description)
                self.assertEqual(app.index, 0)
                self.assertEqual(app.records, [])
                self.assertFalse((app.session / "touches.jsonl").exists())
                # A capture identical to its baseline must advance the guided step.
                app.reader.xy = None
                app.capture()
                settle()
                self.assertEqual(app.state, "touch", app.status.get())
                with patch.object(app, "read_capture", return_value=app.baseline):
                    app.capture()
                    settle()
                self.assertEqual(app.index, 1, app.status.get())
                self.assertEqual(app.records[0]["contact"], app.records[0]["baseline"])
                app.start_training()
                for _ in range(10):
                    app.reader.xy = None
                    app.capture()
                    settle()
                    self.assertEqual(app.state, "touch", app.status.get())
                    app.reader.xy = app.plan[app.index % len(app.plan)]
                    app.capture()
                    settle()
                self.assertEqual(app.state, "ready")
                self.assertEqual(len(app.records), 10)
                self.assertEqual(max(overlap), 1, "Serial preview and saved captures overlapped")
                self.assertEqual(app.signal_plot.summary["frames"], 5)
                app.fit()
                settle()
                self.assertIsNotNone(app.model, app.status.get())
                self.assertEqual(app.model.model_type, self.training_model)
                self.assertEqual(app.model.metadata["eit_config"]["eit_training_model"], self.training_model)
                self.assertEqual(app.model.metadata["allow_clipped"], self.force_clipped)
                self.assertEqual(app.model.metadata["usable_measurement_channels"], channels-int(self.force_zero))
                if self.force_zero:
                    self.assertFalse(app.model.channel_mask[0])
                    self.assertTrue(app.signal_plot.summary["zero_channels"][0])
                if self.force_clipped:
                    self.assertTrue(app.records[0]["contact"]["frames"][0]["clipped"][0])
                    self.assertTrue(app.signal_plot.summary["clipped"][0])
                self.assertEqual(app.model.x.shape[1], 2*channels)
                self.assertEqual(app.model.metadata["drive_pattern"],self.drive_pattern)
                model_path = app.session / ("position_model_mlp.npz" if self.training_model == "mlp"
                                            else "position_model.npz")
                self.assertTrue(model_path.exists())
                if self.training_model == "mlp":
                    original_model = model_path.read_bytes()
                    app.model_type.set("rbf")
                    app.fit()
                    settle()
                    self.assertEqual(app.model.model_type, "rbf", app.status.get())
                    self.assertEqual(app.model.metadata["eit_config"]["eit_training_model"], "rbf")
                    self.assertTrue((app.session / "position_model.npz").exists())
                    self.assertTrue((app.session / "evaluation_mlp.json").exists())
                    self.assertEqual(model_path.read_bytes(), original_model)
                    app.load(model_path)
                    self.assertEqual(app.model_type.get(), "mlp")
                app.reader.xy = None
                app.start_test()
                app.capture()
                deadline = time.monotonic() + 5
                while app.state != "testing" and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.01)
                self.assertEqual(app.state, "testing", app.status.get())
                app.reader.xy = [0, 0]
                deadline = time.monotonic() + 5
                while len(app.prediction.get_xdata()) == 0 and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.02)
                self.assertEqual(len(app.prediction.get_xdata()), 1, app.status.get())
                self.assertIn("RMS change", app.prediction_metrics.get())
                self.assertIn("distance", app.prediction_metrics.get())
                self.assertIn("limit", app.prediction_metrics.get())
                self.assertNotIn("—", app.prediction_metrics.get())
                with patch.object(app.model, "support_limit", -1.):
                    deadline = time.monotonic() + 5
                    while "unfamiliar signal" not in app.status.get() and time.monotonic() < deadline:
                        root.update()
                        time.sleep(.02)
                    self.assertIn("unfamiliar signal", app.status.get())
                    self.assertEqual(len(app.prediction.get_xdata()), 1, app.status.get())
                    self.assertIn("Signal familiarity: unfamiliar", app.prediction_metrics.get())
                app.reader.xy = None
                previous_reading = app.signal_plot.last_sample_s
                deadline = time.monotonic() + 5
                while app.signal_plot.last_sample_s == previous_reading and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.02)
                self.assertGreater(app.signal_plot.last_sample_s, previous_reading)
                self.assertEqual(len(app.prediction.get_xdata()), 1, app.status.get())
                app.stop_test()
                self.assertIn("test stopped", app.prediction_metrics.get())
                self.assertNotIn("RMS change", app.prediction_metrics.get())
                settle()
                app.load(model_path)
                self.assertEqual(app.state, "idle", app.status.get())
                self.assertIn("start Live test", app.prediction_metrics.get())
            finally:
                app.state = "idle"
                app.debug_enabled.set(False)
                settle()
                deadline = time.monotonic() + 5
                while app.debug_busy and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.01)
                for job in root.tk.call("after", "info"):
                    root.after_cancel(job)
                root.destroy()


class SixteenWorkflowTests(WorkflowTests):
    electrode_count = 16


class CombinedWorkflowTests(SixteenWorkflowTests):
    drive_pattern = "combined"


class AllowClippedWorkflowTests(WorkflowTests):
    electrode_count = 16
    force_clipped = True


class ZeroChannelWorkflowTests(AllowClippedWorkflowTests):
    force_zero = True


class MLPWorkflowTests(CombinedWorkflowTests):
    training_model = "mlp"
    force_clipped = True
    force_zero = True


@unittest.skipUnless(os.environ.get("DISPLAY"), "GUI requires a display")
class LegacyProtocolTests(unittest.TestCase):
    def test_old_model_restores_adjacent_scan_and_connected_scan_mismatch_rejected(self):
        import tkinter as tk
        from contact_position_app import ContactApp
        cfg={"eit_electrode_pins":list(range(1,17)),"eit_drive_pattern":"combined",
             "eit_frequency_khz":50,"eit_periods_per_measurement":1,"eit_frames_per_capture":5}
        old_settings={key:value for key,value in cfg.items() if key!="eit_drive_pattern"}
        x=np.random.default_rng(8).normal(size=(3,416))
        x/=np.linalg.norm(x,axis=1)[:,None]
        metadata={"radius_mm":45.,"numbering":"ccw","demo":True,"eit_config":old_settings}
        legacy=PositionModel(x,[[0,0],[10,0],[0,10]],.001,metadata)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"old_adjacent.npz"
            legacy.save(path)
            args=argparse.Namespace(demo=True,points=5,repeats=1,numbering="ccw",radius_mm=45.,
                                    margin_mm=5.,output=Path(folder),model=None)
            root=tk.Tk()
            root.withdraw()
            try:
                app=ContactApp(root,args,dict(cfg))
                self.assertIn("304 measurements/frame",app.scan_summary.get())
                app.reader=DemoReader(cfg)  # Simulate an already configured combined connection.
                app.state="idle"
                app.load(path)
                self.assertIsNone(app.model)
                self.assertIn("different drive pattern",app.status.get())
                app.reader=None
                app.load(path)
                self.assertIsNotNone(app.model,app.status.get())
                self.assertEqual(app.drive_pattern.get(),"adjacent")
                self.assertEqual(app.cfg["eit_drive_pattern"],"adjacent")
                self.assertIn("208 measurements/frame",app.scan_summary.get())
            finally:
                root.update_idletasks()
                for job in root.tk.call("after","info"):
                    root.after_cancel(job)
                root.destroy()


if __name__ == "__main__":
    unittest.main()
