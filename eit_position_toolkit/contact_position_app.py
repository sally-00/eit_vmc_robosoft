"""Guided manual EIT position training and live testing on the numbered plate.

Run with --demo to exercise the UI without a serial device or robot.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import threading
import time

import numpy as np

from contact_position import (HERE, DemoReader, PositionModel, baseline_reference, draw_pad, fit_samples,
                              new_session, signature, targets, voltages)
from eit_reader import EITReader, measurement_protocol, validate_rx_margin
from contact_signal_debug import SignalDebugPlot, SignalChannelTable, save_signal_snapshot
from review_session import review_records


class ContactApp:
    def __init__(self, root, args, cfg):
        import tkinter as tk
        from tkinter import ttk
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

        self.root, self.args, self.cfg = root, args, cfg
        self.reader = None
        self.model = None
        self.session = None
        self.records = []
        self.plan = None
        self.baseline = None
        self.test_baseline = None
        self.index = 0
        self.state = "disconnected"
        self.busy = False
        self.closing = False
        self.results = queue.Queue()
        self.signal_results = queue.Queue()
        self.reader_lock = threading.Lock()
        self.debug_busy = False
        self.next_debug_s = 0.
        root.title("EIT · Contact position" + (" · SYNTHETIC DEMO" if args.demo else ""))
        root.geometry("1400x920")
        self.numbering = tk.StringVar(value=args.numbering)
        self.drive_pattern = tk.StringVar(value=cfg.get("eit_drive_pattern", "adjacent"))
        self.model_type = tk.StringVar(value=getattr(args, "training_model", None) or cfg.get("eit_training_model", "rbf"))
        self.rx_gain_margin = tk.IntVar(value=cfg.get("eit_rx_gain_margin_steps", 0))
        self.scan_summary = tk.StringVar(value="")
        self.pin_summary = tk.StringVar(value="")
        self.count = tk.IntVar(value=args.points)
        self.repeats = tk.IntVar(value=args.repeats)
        self.status = tk.StringVar(value="Remove all contact, then connect to EIT.")
        self.detail = tk.StringVar(value="")
        self.progress = tk.StringVar(value="Position only · no force labels or robot movement")
        self.geometry_label = tk.StringVar(value=f"Active Ø {2*args.radius_mm:g} mm · edge inset {args.margin_mm:g} mm")
        self.debug_enabled = tk.BooleanVar(value=True)
        self.allow_clipped = tk.BooleanVar(value=cfg.get("eit_allow_clipped", False))
        self.signal_status = tk.StringVar(value="Waiting for EIT readings")
        self.prediction_metrics = tk.StringVar()
        self.update_prediction_metrics()

        settings = ttk.Frame(root, padding=10)
        settings.pack(fill="x")
        ttk.Label(settings, text="Training points").pack(side="left")
        self.point_entry = ttk.Spinbox(settings, from_=3, to=500, width=5, textvariable=self.count)
        self.point_entry.pack(side="left", padx=6)
        ttk.Label(settings, text="Touches / point").pack(side="left")
        self.repeat_entry = ttk.Spinbox(settings, from_=1, to=20, width=4, textvariable=self.repeats)
        self.repeat_entry.pack(side="left", padx=6)
        ttk.Label(settings, text="Engraved plate").pack(side="left", padx=(15, 0))
        self.orientation = ttk.Combobox(settings, values=("ccw", "cw"), state="readonly",
                                        width=5, textvariable=self.numbering)
        self.orientation.pack(side="left", padx=6)
        self.orientation.bind("<<ComboboxSelected>>", lambda _: self.redraw())
        ttk.Label(settings, textvariable=self.geometry_label).pack(side="right")
        scan_settings = ttk.Frame(root, padding=(10, 0, 10, 5))
        scan_settings.pack(fill="x")
        ttk.Label(scan_settings, text="Current drive pattern").pack(side="left")
        self.pattern_option = ttk.Combobox(scan_settings, values=("adjacent", "opposite", "combined"),
                                           textvariable=self.drive_pattern, state="readonly", width=12)
        self.pattern_option.pack(side="left", padx=6)
        self.pattern_option.bind("<<ComboboxSelected>>", lambda _: self.update_scan_settings())
        ttk.Label(scan_settings, textvariable=self.scan_summary).pack(side="left", padx=10)
        ttk.Label(scan_settings, text="Training model").pack(side="left", padx=(10, 0))
        self.model_option = ttk.Combobox(scan_settings, values=("rbf", "mlp"),
                                          textvariable=self.model_type, state="readonly", width=7)
        self.model_option.pack(side="left", padx=6)
        gain_settings = ttk.Frame(root, padding=(10, 0, 10, 5))
        gain_settings.pack(fill="x")
        ttk.Label(gain_settings, text="Extra RX gain halvings after autogain").pack(side="left")
        self.gain_margin_option = ttk.Spinbox(gain_settings, from_=0, to=14, width=4,
                                             textvariable=self.rx_gain_margin)
        self.gain_margin_option.pack(side="left", padx=6)
        ttk.Label(gain_settings, text="0 = original · 1 = ½ RX gain · 2 = ¼ RX gain · set before connecting").pack(side="left")
        self.update_scan_settings()
        ttk.Label(root, textvariable=self.pin_summary, wraplength=1300).pack()
        ttk.Label(root, text="Match the physical engraved face to this view before collecting.",
                  font=("sans", 12, "bold")).pack()
        charts = ttk.Frame(root)
        charts.pack(fill="both", expand=True)
        pad_frame = ttk.Frame(charts)
        pad_frame.pack(side="left", fill="both", expand=True)
        signal_frame = ttk.Frame(charts)
        signal_frame.pack(side="left", fill="both", expand=True)
        self.figure = Figure(figsize=(7, 6), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=pad_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas.mpl_connect("button_press_event", self.demo_click)
        self.redraw()
        ttk.Checkbutton(signal_frame, text="Live signal debug (preview readings are not saved)",
                        variable=self.debug_enabled).pack(anchor="w", padx=12)
        self.clipping_option = ttk.Checkbutton(signal_frame, text="Allow clipped readings (continue experiment)",
                                               variable=self.allow_clipped)
        self.clipping_option.pack(anchor="w", padx=12)
        signal_tabs = ttk.Notebook(signal_frame)
        signal_tabs.pack(fill="both",expand=True)
        plot_tab = ttk.Frame(signal_tabs)
        channel_tab = ttk.Frame(signal_tabs)
        signal_tabs.add(plot_tab,text="Signal plots")
        signal_tabs.add(channel_tab,text="Channel diagnostics")
        ttk.Label(channel_tab,text="* Estimated sine limit from active RX gain. Clipped amplitudes cannot measure headroom.\nVariation is within-capture change; during motion it is not a pure noise measurement.",wraplength=570).pack(fill="x",pady=5)
        self.channel_table = SignalChannelTable(channel_tab)
        self.signal_figure = Figure(figsize=(6, 7), dpi=100)
        self.signal_canvas = FigureCanvasTkAgg(self.signal_figure, master=plot_tab)
        self.signal_plot = SignalDebugPlot(self.signal_figure)
        self.signal_plot.adc_peak_limit_v = cfg.get("eit_adc_peak_limit_v",2.047)
        self.signal_plot.warning_percent = cfg.get("eit_range_warning_percent",80.)
        self.signal_plot.low_percent = cfg.get("eit_low_range_percent",5.)
        self.signal_canvas.get_tk_widget().pack(fill="both", expand=True)
        self.signal_label = ttk.Label(signal_frame, textvariable=self.signal_status,
                                      wraplength=580, justify="left")
        self.signal_label.pack(fill="x", padx=10, pady=4)
        ttk.Button(signal_frame,text="Save signal snapshot…",command=self.save_snapshot).pack(anchor="w",padx=10)
        ttk.Label(signal_frame,text="Range is an estimate from the supplied firmware and active RX gain. C overrides a low plotted voltage; it is not a healthy low signal.",wraplength=580).pack(fill="x",padx=10)
        ttk.Label(signal_frame, textvariable=self.prediction_metrics, wraplength=580,
                  justify="left").pack(fill="x", padx=10, pady=4)
        ttk.Label(root, textvariable=self.progress, font=("sans", 12)).pack(pady=3)
        ttk.Label(root, textvariable=self.status, font=("sans", 13, "bold"),
                  wraplength=1000, justify="center").pack(pady=5)
        ttk.Label(root, textvariable=self.detail, wraplength=1000, justify="center").pack(pady=3)
        buttons = ttk.Frame(root, padding=10)
        buttons.pack()
        self.connect_button = ttk.Button(buttons, text="Connect (pad untouched)", command=self.connect)
        self.connect_button.pack(side="left", padx=4)
        self.train_button = ttk.Button(buttons, text="New training", command=self.start_training)
        self.train_button.pack(side="left", padx=4)
        self.capture_button = ttk.Button(buttons, text="Capture", command=self.capture)
        self.capture_button.pack(side="left", padx=4)
        self.fit_button = ttk.Button(buttons, text="Train model", command=self.fit)
        self.fit_button.pack(side="left", padx=4)
        self.load_button = ttk.Button(buttons, text="Load model…", command=self.load)
        self.load_button.pack(side="left", padx=4)
        self.test_button = ttk.Button(buttons, text="Live test", command=self.start_test)
        self.test_button.pack(side="left", padx=4)
        self.stop_button = ttk.Button(buttons, text="Stop test", command=self.stop_test)
        self.stop_button.pack(side="left", padx=4)
        self.demo_release = ttk.Button(buttons, text="Demo: release", command=self.release_demo)
        if args.demo:
            self.demo_release.pack(side="left", padx=4)
            ttk.Label(root, text="SYNTHETIC DEMO: click inside the pad to hold a simulated touch; right-click to release.",
                      foreground="#b05015").pack(pady=4)
        self.refresh()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(50, self.poll)
        if args.model:
            self.load(args.model)

    def save_snapshot(self):
        from tkinter import filedialog
        if self.signal_plot.last_capture is None:
            self.status.set("No current signal capture to save.")
            return
        path=filedialog.asksaveasfilename(title="Save EIT signal snapshot",defaultextension=".json",
            initialfile="eit_signal_"+datetime.now().strftime("%Y%m%d_%H%M%S")+".json",filetypes=[("Signal snapshot","*.json")])
        if path:
            try:
                save_signal_snapshot(path,self.signal_plot)
                self.detail.set("Signal snapshot saved: "+path)
            except Exception as exc:self.detail.set("Cannot save signal snapshot: "+str(exc))

    def redraw(self):
        self.ax.clear()
        draw_pad(self.ax, self.numbering.get(), self.args.radius_mm)
        self.pending, = self.ax.plot([], [], ".", color="#c0b9a8", ms=5)
        self.done, = self.ax.plot([], [], "o", color="#378f77", ms=5)
        self.target, = self.ax.plot([], [], "+", color="#d27610", ms=25, mew=3)
        self.prediction, = self.ax.plot([], [], "o", color="#db3b59", ms=15, alpha=.85)
        self.demo_marker, = self.ax.plot([], [], "x", color="#616c77", ms=9)
        self.figure.tight_layout()
        self.canvas.draw_idle()

    def refresh(self):
        def enabled(widget, condition):
            widget.configure(state="normal" if condition else "disabled")
        idle = not self.busy and not self.closing
        collecting = self.state in ("release", "touch", "ready")
        enabled(self.connect_button, idle and self.reader is None)
        enabled(self.train_button, idle and self.reader is not None and not collecting and self.state != "testing")
        enabled(self.capture_button, idle and self.state in ("release", "touch", "test_release"))
        enabled(self.fit_button, idle and (self.state == "ready" or (self.state == "idle" and bool(self.records))))
        enabled(self.load_button, idle and not collecting and self.state != "testing")
        enabled(self.test_button, idle and self.model is not None and self.reader is not None and not collecting and self.state != "testing")
        enabled(self.stop_button, self.state in ("testing", "test_release"))
        enabled(self.point_entry, idle and not collecting and self.state != "testing")
        enabled(self.repeat_entry, idle and not collecting and self.state != "testing")
        enabled(self.clipping_option, idle and not collecting and self.state not in ("testing", "test_release"))
        self.orientation.configure(state="readonly" if idle and self.reader is None and self.model is None else "disabled")
        self.pattern_option.configure(state="readonly" if idle and self.reader is None and self.model is None else "disabled")
        self.model_option.configure(state="readonly" if idle and self.state not in
                                    ("release", "touch", "testing", "test_release") else "disabled")
        enabled(self.gain_margin_option, idle and self.reader is None)

    def update_scan_settings(self):
        self.cfg["eit_drive_pattern"] = self.drive_pattern.get()
        pins = self.cfg.get("eit_electrode_pins")
        if pins is None:
            self.scan_summary.set("Configure electrode pins before connecting")
            self.pin_summary.set("Active electrode pins: not configured")
            return
        channels = len(measurement_protocol(pins, self.drive_pattern.get()))
        self.scan_summary.set(f"{len(pins)} electrodes · {channels} measurements/frame · {2*channels} model inputs")
        self.pin_summary.set("Active electrode pins: " + ", ".join(map(str, pins)))

    def work(self, function, callback):
        if self.busy:
            return
        self.busy = True
        self.refresh()
        def run():
            try:
                self.results.put((callback, function(), None))
            except Exception as error:
                self.results.put((callback, None, error))
        threading.Thread(target=run, daemon=True).start()

    def poll(self):
        try:
            callback, result, error = self.results.get_nowait()
        except queue.Empty:
            pass
        else:
            self.busy = False
            if self.closing and isinstance(result, (EITReader, DemoReader)):
                result.close()
            if not self.closing:
                try:
                    if error:
                        raise error
                    callback(result)
                except Exception as exc:
                    if self.state == "testing":
                        self.state = "idle"
                    self.prediction.set_data([], [])
                    self.update_prediction_metrics(reason="unavailable after capture failure")
                    self.status.set("Capture failed: " + str(exc))
                    self.detail.set("No touch was saved. Correct the issue and retry the current step.")
                self.refresh()
                self.canvas.draw_idle()
        while True:
            try:
                value, reference, source, completed_s, error, preview = self.signal_results.get_nowait()
            except queue.Empty:
                break
            if preview:
                self.debug_busy = False
            if not self.closing:
                if error:
                    self.signal_plot.unavailable(error)
                else:
                    try:
                        self.signal_plot.update(value, reference, source, completed_s)
                    except Exception as exc:
                        self.signal_plot.unavailable(exc)
        if not self.closing:
            label = self.signal_plot.description
            if self.signal_plot.last_sample_s is not None:
                label += f" · last reading {time.monotonic()-self.signal_plot.last_sample_s:.1f}s ago"
            if not self.debug_enabled.get() and self.state != "testing":
                label = "Preview paused · " + label
            self.signal_status.set(label)
            summary = self.signal_plot.summary
            self.channel_table.update(summary,self.signal_plot.device,self.signal_plot.warning_percent,self.signal_plot.low_percent)
            self.signal_label.configure(foreground="#b3261e" if "unavailable" in label or
                                        (summary is not None and summary["clipped"].any()) else "#234955")
            self.debug_step()
        if self.closing and not self.busy and not self.debug_busy:
            try:
                if self.reader:
                    self.reader.close()
            finally:
                self.root.destroy()
            return
        self.root.after(50, self.poll)

    def read_capture(self, source, reference=None, baseline_capture=False, preview=False):
        """All serial acquisitions share one lock; preview never consumes a saved capture."""
        try:
            with self.reader_lock:
                value = self.reader.capture()
                completed_s = time.monotonic()
        except Exception as exc:
            self.signal_results.put((None, None, source, time.monotonic(), exc, preview))
            raise
        self.signal_results.put((value, value if baseline_capture else reference,
                                 source, completed_s, None, preview))
        return value

    def debug_step(self):
        if (not self.debug_enabled.get() or self.reader is None or self.busy or self.debug_busy
                or self.state == "testing" or time.monotonic() < self.next_debug_s):
            return
        self.debug_busy = True
        self.next_debug_s = time.monotonic() + .25
        reference = self.test_baseline if self.state == "test_release" else self.baseline
        def preview():
            try:
                self.read_capture("Live preview · not saved", reference, preview=True)
            except Exception:
                pass  # The signal queue reports the error without altering the guided step.
        threading.Thread(target=preview, daemon=True).start()

    def connect(self):
        if self.reader is not None:
            return
        try:
            self.cfg["eit_rx_gain_margin_steps"] = validate_rx_margin(self.rx_gain_margin.get())
        except Exception as exc:
            self.status.set(str(exc))
            return
        self.status.set("Connecting and setting gain… keep the pad untouched.")
        allow_clipped = self.allow_clipped.get()
        def open_reader():
            reader = DemoReader(self.cfg) if self.args.demo else EITReader(self.cfg)
            try:
                reader.open()
                baseline = reader.capture()
                self.signal_results.put((baseline, None, "Connected · untouched pad", time.monotonic(), None, False))
                baseline_reference(voltages(baseline, allow_clipped))
            except Exception:
                reader.close()
                raise
            return reader
        def connected(reader):
            self.reader = reader
            self.signal_plot.device = reader.metadata()
            self.state = "idle"
            channels = len(reader.protocol)
            self.status.set(f"Connected: {self.drive_pattern.get()} drive · {len(self.cfg['eit_electrode_pins'])} electrodes · {channels} measurements/frame · extra RX halvings {self.rx_gain_margin.get()}. Start training or load a model.")
            self.detail.set("CCW: 1 bottom, 3 right, 5 top, 7 left. CW: 1 top, 3 right, 5 bottom, 7 left.")
        self.work(open_reader, connected)

    def start_training(self):
        try:
            count, repeats = self.count.get(), self.repeats.get()
            if not 1 <= repeats <= 20:
                raise ValueError("Choose 1–20 touches per point")
            plan = targets(count, self.args.radius_mm, self.args.margin_mm)
            metadata = {"created_utc": datetime.now(timezone.utc).isoformat(),
                        "demo": self.args.demo, "numbering": self.numbering.get(),
                        "radius_mm": self.args.radius_mm, "margin_mm": self.args.margin_mm,
                        "point_count": count, "repetitions": repeats,
                        "model_type": self.model_type.get(),
                        "allow_clipped": self.allow_clipped.get(),
                        "contact_strength_check": False,
                        "unfamiliar_predictions": True,
                        "targets_mm": plan.tolist(),
                        "eit_config": {**self.cfg, "eit_training_model": self.model_type.get()},
                        "device": self.reader.metadata(), "label": "contact position only, millimetres",
                        "electrode_count": len(self.cfg["eit_electrode_pins"]),
                        "drive_pattern": self.drive_pattern.get(),
                        "measurement_channels": len(self.reader.protocol)}
            session = new_session(self.args.output, metadata)
        except Exception as exc:
            self.status.set(str(exc))
            return
        self.session, self.metadata, self.plan = session, metadata, plan
        self.training_repeats = repeats
        self.records, self.index = [], 0
        self.baseline = None
        self.test_baseline = None
        self.signal_plot.reset_reference()
        self.model = None
        self.update_prediction_metrics(reason="available during Live test after training")
        self.prediction.set_data([], [])
        self.done.set_data([], [])
        self.pending.set_data(plan[:, 0], plan[:, 1])
        self.next_touch()

    def next_touch(self):
        if self.index == len(self.plan) * self.training_repeats:
            self.state = "ready"
            self.target.set_data([], [])
            self.status.set("All touches saved. Select Train model.")
            self.capture_button.configure(text="Capture")
            return
        point = self.index % len(self.plan)
        repeat = self.index // len(self.plan)
        self.target.set_data([], [])
        self.state = "release"
        self.progress.set(f"Point {point+1}/{len(self.plan)} · round {repeat+1}/{self.training_repeats} · {self.index} touches saved")
        self.status.set("Lift off the pad completely. Capture the untouched baseline.")
        self.detail.set("Keep the pad untouched. The orange target appears after the baseline capture finishes.")
        self.capture_button.configure(text="Capture untouched baseline")
        self.refresh()
        self.canvas.draw_idle()

    def capture(self):
        stage = self.state
        allow_clipped = self.allow_clipped.get()
        def captured(value):
            voltages(value, allow_clipped)
            if stage == "release":
                baseline_reference(voltages(value, allow_clipped))
                self.baseline = value
                self.state = "touch"
                xy = self.plan[self.index % len(self.plan)]
                self.target.set_data([xy[0]], [xy[1]])
                self.status.set("Touch the orange cross, hold still, then capture the touch.")
                self.detail.set(f"Touch X {xy[0]:+.1f}, Y {xy[1]:+.1f} mm. Match the engraved numbers.")
                self.capture_button.configure(text="Capture held touch")
            elif stage == "touch":
                signature(value, self.baseline, allow_clipped)
                point = self.index % len(self.plan)
                record = {"point_id": point, "repeat": self.index // len(self.plan),
                          "xy_mm": self.plan[point].tolist(), "baseline": self.baseline, "contact": value}
                with (self.session / "touches.jsonl").open("a") as stream:
                    stream.write(json.dumps(record) + "\n")
                self.records.append(record)
                self.index += 1
                completed = self.plan[:min(self.index, len(self.plan))]
                self.done.set_data(completed[:, 0], completed[:, 1])
                self.next_touch()
            elif stage == "test_release" and self.state == "test_release":
                baseline_reference(voltages(value, allow_clipped), self.model.channel_mask)
                self.test_baseline = value
                self.state = "testing"
                self.status.set(f"Live {self.model.model_type.upper()} test active. Poke anywhere in the trained area.")
                self.detail.set("Pink dot = estimated position. Strength checks are off; the dot may remain after release.")
                self.live_step()
        is_baseline = stage in ("release", "test_release")
        source = "Untouched baseline capture" if is_baseline else "Held touch capture"
        self.work(lambda: self.read_capture(source, self.baseline, baseline_capture=is_baseline), captured)

    def fit(self):
        model_type = self.model_type.get()
        metadata = {**self.metadata, "model_type": model_type,
                    "eit_config": {**self.metadata["eit_config"], "eit_training_model": model_type}}
        model_name = "position_model_mlp.npz" if model_type == "mlp" else "position_model.npz"
        evaluation_name = "evaluation_mlp.json" if model_type == "mlp" else "evaluation.json"
        self.status.set(f"Training {model_type.upper()} position model…")
        def train():
            quality = review_records(self.records, metadata)
            (self.session / "signal_quality.json").write_text(json.dumps(quality,indent=2,allow_nan=False)+"\n")
            model, report = fit_samples(self.records, metadata)
            report["signal_advisory_touches"] = quality["touches_with_advisories"]
            model.save(self.session / model_name)
            (self.session / "manifest.json").write_text(json.dumps(model.metadata, indent=2))
            (self.session / evaluation_name).write_text(json.dumps(report, indent=2))
            return model, report
        def trained(result):
            self.model, report = result
            self.state = "idle"
            self.status.set(f"{self.model.model_type.upper()} position model saved. Select Live test.")
            if "median_error_mm" in report:
                self.detail.set(f"Held-out touches at trained locations: median error {report['median_error_mm']:.1f} mm. "
                                "Accuracy between locations still needs a physical check.")
            else:
                self.detail.set(report["validation"])
            self.detail.set(self.detail.get() + f" Using {self.model.metadata['usable_measurement_channels']}/"
                            f"{self.model.metadata['measurement_channels']} measurement channels.")
            self.detail.set(self.detail.get()+f" Signal advisories: {report.get('signal_advisory_touches',0)}/{len(self.records)} touches; see signal_quality.json.")
            self.progress.set(str(self.session / model_name))
        self.work(train, trained)

    def load(self, path=None):
        from tkinter import filedialog
        path = path or filedialog.askopenfilename(title="Load a contact-position model", filetypes=[("Position models", "*.npz")])
        if not path:
            return
        try:
            model = PositionModel.load(path)
            if bool(model.metadata.get("demo")) != self.args.demo:
                raise ValueError("Demo and hardware models must be used in their matching mode")
            saved = model.metadata["eit_config"]
            if self.reader is not None:
                keys = ("eit_electrode_pins", "eit_frequency_khz", "eit_periods_per_measurement")
                if any(saved.get(k) != self.cfg.get(k) for k in keys):
                    raise ValueError("Model requires different EIT settings. Restart with --model PATH before connecting")
                if saved.get("eit_drive_pattern", "adjacent") != self.cfg.get("eit_drive_pattern", "adjacent"):
                    raise ValueError("Model uses a different drive pattern. Retrain for this scan or restart with --model PATH")
            else:
                self.cfg.update({k: saved[k] for k in ("eit_electrode_pins", "eit_frequency_khz", "eit_periods_per_measurement")})
                self.cfg["eit_drive_pattern"] = saved.get("eit_drive_pattern", "adjacent")
            self.drive_pattern.set(self.cfg.get("eit_drive_pattern", "adjacent"))
            self.update_scan_settings()
            self.numbering.set(model.metadata["numbering"])
            self.args.radius_mm = model.metadata["radius_mm"]
            self.args.margin_mm = model.metadata.get("margin_mm", self.args.margin_mm)
            self.geometry_label.set(f"Active Ø {2*self.args.radius_mm:g} mm · edge inset {self.args.margin_mm:g} mm")
            self.model = model
            self.model_type.set(model.model_type)
            self.records = []
            self.update_prediction_metrics()
            self.plan = None
            self.state = "idle" if self.reader else "disconnected"
            self.redraw()
            self.progress.set(str(path))
            self.status.set(f"{model.model_type.upper()} model loaded: {self.drive_pattern.get()} drive · {model.metadata['measurement_channels']} measurements/frame. "
                            + ("Select Live test." if self.reader else "Connect with the pad untouched."))
            pins = ", ".join(map(str, saved["eit_electrode_pins"]))
            requested_pins = getattr(self.args, "pins", None)
            if requested_pins is not None and requested_pins != saved["eit_electrode_pins"]:
                self.detail.set(f"Using the model's saved pins {pins}, overriding --pins. Match the physical wiring to this model.")
            else:
                self.detail.set(f"Saved model pins: {pins}. Plate orientation and EIT scan settings are restored with the model.")
        except Exception as exc:
            self.status.set("Cannot load model: " + str(exc))
        self.refresh()

    def start_test(self):
        self.model_type.set(self.model.model_type)
        self.state = "test_release"
        self.test_baseline = None
        self.update_prediction_metrics(reason="capture the untouched test baseline")
        self.signal_plot.reset_reference()
        self.target.set_data([], [])
        self.prediction.set_data([], [])
        self.status.set("Lift off the pad, then capture a fresh untouched baseline for testing.")
        self.capture_button.configure(text="Capture test baseline")
        self.refresh()
        self.canvas.draw_idle()

    def update_prediction_metrics(self, result=None, reason="start Live test and capture a baseline"):
        if result is None:
            self.prediction_metrics.set(f"Signal strength: —\nSignal familiarity: — ({reason})")
            return
        strength = result["strength_rms_percent"]
        distance, limit = result["pattern_distance"], result["pattern_limit"]
        familiarity = "familiar" if result["pattern_familiar"] else "unfamiliar"
        extra = " · predicted position outside pad" if not result["within_pad"] else ""
        self.prediction_metrics.set(
            f"Signal strength: RMS change {strength:.3f}% · combined score {result['strength']:.4g}\n"
            f"Signal familiarity: {familiarity} · distance {distance:.3f} / limit {limit:.3f}{extra}\n"
            "Lower distance = closer to training examples; not a confidence percentage.")

    def live_step(self):
        if self.state != "testing" or self.closing:
            return
        def predicted(capture):
            if self.state != "testing":
                return
            result = self.model.predict(capture, self.test_baseline, allow_clipped=self.allow_clipped.get())
            self.update_prediction_metrics(result)
            xy = result["xy_mm"]
            self.prediction.set_data([xy[0]], [xy[1]]) if xy is not None else self.prediction.set_data([], [])
            if xy is not None:
                label = " · unfamiliar signal" if result["state"] == "unfamiliar" else ""
                self.status.set(f"Position: X {xy[0]:+.1f} mm · Y {xy[1]:+.1f} mm{label}")
            else:
                self.status.set("Position unavailable")
            self.root.after(50, self.live_step)
        self.work(lambda: self.read_capture("Live model test", self.test_baseline), predicted)

    def stop_test(self):
        self.state = "idle"
        self.update_prediction_metrics(reason="test stopped")
        self.prediction.set_data([], [])
        self.status.set("Test stopped. Select Live test to capture a new untouched baseline.")
        self.refresh()
        self.canvas.draw_idle()

    def demo_click(self, event):
        if not self.args.demo or self.reader is None or event.inaxes != self.ax:
            return
        if event.button == 3:
            self.release_demo()
        elif event.button == 1 and np.hypot(event.xdata, event.ydata) <= self.args.radius_mm:
            self.reader.xy = [event.xdata, event.ydata]
            self.demo_marker.set_data([event.xdata], [event.ydata])
            self.canvas.draw_idle()

    def release_demo(self):
        if self.reader:
            self.reader.xy = None
        self.demo_marker.set_data([], [])
        self.canvas.draw_idle()

    def close(self):
        self.closing = True
        self.state = "closing"
        self.status.set("Closing EIT connection…")
        self.refresh()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=HERE / "config.json")
    parser.add_argument("--points", type=int, default=20)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--radius-mm", type=float, default=None)
    parser.add_argument("--margin-mm", type=float, default=5.)
    parser.add_argument("--numbering", choices=("ccw", "cw"), default="ccw")
    parser.add_argument("--port", help="USB serial port; otherwise auto-detect Amodo VID/PID")
    parser.add_argument("--pins", type=int, nargs="+", metavar="PIN", help="8 or 16 device pins in physical order around the pad; independent of STL landmark numbers")
    parser.add_argument("--drive-pattern", choices=("adjacent", "opposite", "combined"), default=None,
                        help="Current drive pairs; combined appends opposite drives to the adjacent scan (default: config)")
    parser.add_argument("--demo", action="store_true", help="Synthetic signals only, no hardware")
    parser.add_argument("--allow-clipped", action=argparse.BooleanOptionalAction, default=None,
                        help="Accept clipped readings in the trainer; --no-allow-clipped rejects them (default: config)")
    parser.add_argument("--model", type=Path)
    parser.add_argument("--training-model", choices=("rbf", "mlp"), default=None,
                        help="Position regressor for new training (default: config, otherwise rbf)")
    parser.add_argument("--rx-gain-margin-steps", type=int, default=None, metavar="0-14",
                        help="Extra receive-gain halvings after autogain; 1 halves RX, 2 quarters it (default: config)")
    parser.add_argument("--output", type=Path, default=HERE / "contact_position_runs")
    parser.add_argument("--preview", type=Path, help="Save a training-layout PNG and exit without opening hardware")
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    if args.training_model is not None:
        cfg["eit_training_model"] = args.training_model
    if args.rx_gain_margin_steps is not None:
        cfg["eit_rx_gain_margin_steps"] = args.rx_gain_margin_steps
    if cfg.get("eit_training_model", "rbf") not in ("rbf", "mlp"):
        parser.error("eit_training_model must be rbf or mlp")
    if args.allow_clipped is not None:
        cfg["eit_allow_clipped"] = args.allow_clipped
    args.radius_mm = args.radius_mm if args.radius_mm is not None else cfg.get("active_radius_m", .045) * 1000
    try:
        validate_rx_margin(cfg.get("eit_rx_gain_margin_steps", 0))
        plan = targets(args.points, args.radius_mm, args.margin_mm)
        if not 1 <= args.repeats <= 20:
            raise ValueError("Choose 1–20 touches per point")
        if args.port:
            cfg["eit_port"] = args.port
        if args.pins:
            cfg["eit_electrode_pins"] = args.pins
        if args.drive_pattern is not None:
            cfg["eit_drive_pattern"] = args.drive_pattern
        if args.demo and cfg.get("eit_electrode_pins") is None:
            cfg["eit_electrode_pins"] = list(range(1, 17))
        if args.preview:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(8, 8))
            draw_pad(ax, args.numbering, args.radius_mm)
            ax.scatter(plan[:, 0], plan[:, 1], s=35, color="#d27610")
            for i, xy in enumerate(plan):
                ax.annotate(str(i+1), xy, xytext=(4, 4), textcoords="offset points", fontsize=8)
            fig.suptitle(f"EIT position training · {args.points} targets · {args.repeats} touches per target")
            fig.tight_layout()
            fig.savefig(args.preview, dpi=160)
            print(args.preview.resolve())
            return
        # --model restores its saved mapping before a connection is opened.
        if not args.model:
            measurement_protocol(cfg.get("eit_electrode_pins"), cfg.get("eit_drive_pattern", "adjacent"))
    except ValueError as exc:
        parser.error(str(exc))
    import tkinter as tk
    root = tk.Tk()
    ContactApp(root, args, cfg)
    root.mainloop()


if __name__ == "__main__":
    main()
