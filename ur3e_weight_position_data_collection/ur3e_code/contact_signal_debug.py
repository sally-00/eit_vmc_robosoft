"""Raw EIT diagnostics; clipped readings remain visible but are never made valid."""
from collections import deque

import numpy as np


def signal_summary(capture, baseline=None):
    frames = capture["frames"]
    magnitude = np.asarray([f["magnitude_Vrms"] for f in frames], float)
    phase = np.asarray([f["phase_rad"] for f in frames], float)
    clipped = np.asarray([f["clipped"] for f in frames], bool)
    if (magnitude.ndim != 2 or not len(magnitude) or not magnitude.shape[1]
            or phase.shape != magnitude.shape or clipped.shape != magnitude.shape
            or not np.isfinite(magnitude).all() or not np.isfinite(phase).all()
            or np.any(magnitude < 0)):
        raise ValueError("Expected matching finite magnitude/phase/clipping arrays per frame")
    channels = magnitude.shape[1]
    voltage = np.mean(magnitude * np.exp(1j * phase), axis=0)
    # Circular mean avoids a spurious phase of zero across the -pi/pi boundary.
    phase_mean = np.angle(np.mean(np.exp(1j * phase), axis=0))
    relative = np.full(channels, complex(np.nan, np.nan))
    baseline_magnitude = np.full(channels, np.nan)
    baseline_phase = np.full(channels, np.nan)
    if baseline is not None:
        reference = signal_summary(baseline)
        if len(reference["voltage"]) != channels:
            raise ValueError("Debug baseline and current channel counts differ")
        baseline_magnitude = reference["magnitude_mV"]
        baseline_phase = reference["phase_rad"]
        good = np.abs(reference["voltage"]) > 1e-12
        relative[good] = (voltage[good] - reference["voltage"][good]) / reference["voltage"][good]
    return {"magnitude_mV": magnitude.mean(axis=0) * 1000, "phase_rad": phase_mean,
            "voltage": voltage, "clipped": clipped.any(axis=0), "frames": len(frames),
            "zero_channels": np.any(magnitude <= 1e-12, axis=0),
            "baseline_magnitude_mV": baseline_magnitude, "baseline_phase_rad": baseline_phase,
            "relative_percent": relative * 100,
            "rms_change_percent": float(np.sqrt(np.mean(np.abs(relative) ** 2)) * 100)}


class SignalDebugPlot:
    def __init__(self, figure):
        self.figure = figure
        self.axes = figure.subplots(4, 1)
        self.last_sample_s = None
        self.reference_epoch_s = float("-inf")
        self.origin_s = None
        self.history = deque(maxlen=300)
        self.summary = None
        self.description = "Waiting for EIT readings"
        self.channels = np.array([], dtype=int)
        for ax in self.axes[:3]:
            ax.grid(alpha=.2)
            ax.tick_params(labelsize=8)
        magnitude, phase, change, history = self.axes
        magnitude.set_title("Raw voltage · waiting for measurement channels", fontsize=10)
        magnitude.set_ylabel("mV RMS", fontsize=9)
        phase.set_ylabel("Phase (rad)", fontsize=9)
        change.set_ylabel("Change (%)", fontsize=9)
        change.set_xlabel("Measurement channel (not electrode number)", fontsize=8)
        history.set_ylabel("RMS change (%)", fontsize=9)
        history.set_xlabel("Time since first reading (s) · last 30 s", fontsize=8)
        history.grid(alpha=.2)
        history.tick_params(labelsize=8)
        self.magnitude, = magnitude.plot([], [], color="#20779a", lw=1.4, label="Current")
        self.baseline_magnitude, = magnitude.plot([], [], "--", color="#9a8460", lw=1, label="Baseline")
        self.clipped, = magnitude.plot([], [], "rx", ms=7, label="Clipped")
        self.zeros, = magnitude.plot([], [], "o", mfc="none", mec="#cd7c17", ms=6, label="Zero in capture")
        self.phase, = phase.plot([], [], color="#20779a", lw=1.2)
        self.baseline_phase, = phase.plot([], [], "--", color="#9a8460", lw=1)
        self.real, = change.plot([], [], color="#bd5d19", lw=1.2, label="Real")
        self.imaginary, = change.plot([], [], color="#7352a3", lw=1.2, label="Imaginary")
        self.strength, = history.plot([], [], color="#bd5d19", lw=1.4)
        magnitude.legend(loc="upper right", fontsize=7, ncol=2)
        change.legend(loc="upper right", fontsize=7, ncol=2)
        figure.subplots_adjust(left=.15, right=.97, top=.94, bottom=.08, hspace=.55)

    def update(self, capture, baseline, source, completed_s):
        if (completed_s < self.reference_epoch_s
                or (self.last_sample_s is not None and completed_s < self.last_sample_s)):
            return  # Do not overwrite a saved capture with an older queued preview.
        summary = signal_summary(capture, baseline)
        channels = len(summary["magnitude_mV"])
        if channels != len(self.channels):
            self.channels = np.arange(1, channels + 1)
            for ax in self.axes[:3]:
                ax.set_xlim(.5, channels + .5)
                ax.set_xticks(np.unique(np.linspace(1, channels, min(6, channels), dtype=int)))
            self.axes[0].set_title(f"Raw voltage · {channels} measurement channels", fontsize=10)
        if baseline is capture:
            self.reset_reference(completed_s)
        self.summary = summary
        self.last_sample_s = completed_s
        if self.origin_s is None:
            self.origin_s = completed_s
        self.magnitude.set_data(self.channels, summary["magnitude_mV"])
        self.baseline_magnitude.set_data(self.channels, summary["baseline_magnitude_mV"])
        clipped = summary["clipped"]
        self.clipped.set_data(self.channels[clipped], summary["magnitude_mV"][clipped])
        zero_channels = summary["zero_channels"]
        self.zeros.set_data(self.channels[zero_channels], summary["magnitude_mV"][zero_channels])
        self.phase.set_data(self.channels, summary["phase_rad"])
        self.baseline_phase.set_data(self.channels, summary["baseline_phase_rad"])
        self.real.set_data(self.channels, summary["relative_percent"].real)
        self.imaginary.set_data(self.channels, summary["relative_percent"].imag)
        elapsed = completed_s - self.origin_s
        self.history.append((elapsed, summary["rms_change_percent"]))
        while self.history and elapsed - self.history[0][0] > 30:
            self.history.popleft()
        self.strength.set_data(*zip(*self.history))
        for ax in self.axes:
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
        self.axes[3].set_xlim(max(0, elapsed - 30), max(5, elapsed + .2))
        count = int(clipped.sum())
        zeros = int(summary["zero_channels"].sum())
        relative = summary["rms_change_percent"]
        change_text = f"RMS change {relative:.3f}%" if np.isfinite(relative) else "Capture baseline to see changes"
        self.description = f"{source} · {summary['frames']} frames · {change_text} · clipped {count}/{channels}"
        if zeros:
            indices = ", ".join(map(str, np.flatnonzero(zero_channels)[:8] + 1))
            self.description += f" · zero {zeros}/{channels}: channels {indices}" + ("…" if zeros > 8 else "")
        discarded = capture.get("discarded_invalid_frames", 0)
        if discarded:
            self.description += f" · discarded incomplete/invalid frames: {discarded}"
        self.figure.canvas.draw_idle()

    def unavailable(self, message):
        # Clear old channel readings so a failed acquisition never looks live.
        for line in (self.magnitude, self.phase, self.real, self.imaginary, self.clipped, self.zeros):
            line.set_data([], [])
        self.description = "Signal unavailable: " + str(message)
        self.figure.canvas.draw_idle()

    def reset_reference(self, completed_s=None):
        # A newly requested baseline defines a new comparison; never join old/new traces.
        if completed_s is None:
            import time
            completed_s = time.monotonic()
        self.reference_epoch_s = completed_s
        self.history.clear()
        for line in (self.baseline_magnitude, self.baseline_phase, self.real, self.imaginary, self.strength):
            line.set_data([], [])
        self.figure.canvas.draw_idle()
