"""Raw EIT diagnostics; clipped readings remain visible but are never made valid."""
from collections import deque

import numpy as np
from signal_quality import receiver_limits, quality_flags


def signal_summary(capture, baseline=None, device=None, adc_peak_limit_v=2.047):
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
    rx_gain, limits = receiver_limits(device, channels, adc_peak_limit_v)
    usage = 100 * magnitude.max(axis=0) / limits
    complex_frames = magnitude * np.exp(1j * phase)
    frame_noise = np.full(channels, np.nan)
    usable = np.abs(voltage) > 1e-12
    frame_noise[usable] = 100 * np.sqrt(np.mean(np.abs(complex_frames[:, usable] - voltage[usable]) ** 2, axis=0)) / np.abs(voltage[usable])
    # A clipped amplitude is unreliable: do not present its ratio as valid headroom.
    usage[clipped.any(axis=0)] = np.nan
    finite_relative = relative[np.isfinite(relative)]
    rms_change = float(np.sqrt(np.mean(np.abs(finite_relative) ** 2))*100) if len(finite_relative) else float("nan")
    return {"rx_gain": rx_gain, "limit_mV": limits * 1000, "usage_percent": usage,
            "min_mV": magnitude.min(axis=0)*1000, "max_mV": magnitude.max(axis=0)*1000,
            "clipped_fraction": clipped.mean(axis=0), "zero_fraction": (magnitude <= 1e-12).mean(axis=0),
            "frame_noise_percent": frame_noise,
            "magnitude_mV": magnitude.mean(axis=0) * 1000, "phase_rad": phase_mean,
            "voltage": voltage, "clipped": clipped.any(axis=0), "frames": len(frames),
            "zero_channels": np.any(magnitude <= 1e-12, axis=0),
            "baseline_magnitude_mV": baseline_magnitude, "baseline_phase_rad": baseline_phase,
            "relative_percent": relative * 100,
            "rms_change_percent": rms_change}


class SignalDebugPlot:
    def __init__(self, figure):
        self.figure = figure
        self.axes = figure.subplots(5, 1)
        self.device = None
        self.adc_peak_limit_v = 2.047
        self.warning_percent = 80.
        self.low_percent = 5.
        self.last_capture = None
        self.last_baseline = None
        self.last_sample_s = None
        self.reference_epoch_s = float("-inf")
        self.origin_s = None
        self.history = deque(maxlen=300)
        self.summary = None
        self.description = "Waiting for EIT readings"
        self.channels = np.array([], dtype=int)
        for ax in self.axes[:4]:
            ax.grid(alpha=.2)
            ax.tick_params(labelsize=8)
        magnitude, phase, change, usage, history = self.axes
        magnitude.set_title("Raw voltage · waiting for measurement channels", fontsize=10)
        magnitude.set_ylabel("mV RMS", fontsize=9)
        phase.set_ylabel("Phase (rad)", fontsize=9)
        change.set_ylabel("Change (%)", fontsize=9)
        change.set_xlabel("Measurement channel (not electrode number)", fontsize=8)
        usage.set_ylabel("Sine range (%)", fontsize=9)
        usage.axhline(100, color="#000000", ls="--", lw=1, label="Estimated ceiling")
        self.warning_line = usage.axhline(80, color="#E69F00", ls=":", lw=1)
        usage.set_ylim(-5, 115)
        self.usage, = usage.plot([], [], color="#0072B2", lw=1)
        self.usage_clipped, = usage.plot([], [], "x", color="#D55E00", ms=6, label="Clipped (unreliable)")
        self.usage_zero, = usage.plot([], [], "o", mfc="none", mec="#E69F00", ms=5)
        history.set_ylabel("RMS change (%)", fontsize=9)
        history.set_xlabel("Time since first reading (s) · last 30 s", fontsize=8)
        history.grid(alpha=.2)
        history.tick_params(labelsize=8)
        self.magnitude, = magnitude.plot([], [], color="#20779a", lw=1.4, label="Current")
        self.ceiling, = magnitude.plot([], [], "--", color="#000000", lw=.9, label="Estimated sine limit")
        self.baseline_magnitude, = magnitude.plot([], [], "--", color="#9a8460", lw=1, label="Baseline")
        self.clipped, = magnitude.plot([], [], "x", color="#D55E00", ms=7, label="Clipped")
        self.zeros, = magnitude.plot([], [], "o", mfc="none", mec="#cd7c17", ms=6, label="Zero in capture")
        self.phase, = phase.plot([], [], color="#20779a", lw=1.2)
        self.baseline_phase, = phase.plot([], [], "--", color="#9a8460", lw=1)
        self.real, = change.plot([], [], color="#bd5d19", lw=1.2, label="Real")
        self.imaginary, = change.plot([], [], color="#7352a3", lw=1.2, label="Imaginary")
        self.strength, = history.plot([], [], color="#bd5d19", lw=1.4)
        magnitude.legend(loc="upper right", fontsize=6, ncol=3)
        usage.legend(loc="upper right", fontsize=6, ncol=2)
        change.legend(loc="upper right", fontsize=7, ncol=2)
        figure.subplots_adjust(left=.15, right=.97, top=.95, bottom=.07, hspace=.7)

    def update(self, capture, baseline, source, completed_s):
        if (completed_s < self.reference_epoch_s
                or (self.last_sample_s is not None and completed_s < self.last_sample_s)):
            return  # Do not overwrite a saved capture with an older queued preview.
        summary = signal_summary(capture, baseline, self.device, self.adc_peak_limit_v)
        self.last_capture, self.last_baseline = capture, baseline
        channels = len(summary["magnitude_mV"])
        if channels != len(self.channels):
            self.channels = np.arange(1, channels + 1)
            for ax in self.axes[:4]:
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
        self.ceiling.set_data(self.channels, summary["limit_mV"])
        self.usage.set_data(self.channels, summary["usage_percent"])
        self.usage_clipped.set_data(self.channels[summary["clipped"]], np.full(int(summary["clipped"].sum()), 107.))
        self.usage_zero.set_data(self.channels[summary["zero_channels"]], np.zeros(int(summary["zero_channels"].sum())))
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
        for ax in (self.axes[0],self.axes[1],self.axes[2],self.axes[4]):
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
        self.axes[4].set_xlim(max(0, elapsed - 30), max(5, elapsed + .2))
        count = int(clipped.sum())
        zeros = int(summary["zero_channels"].sum())
        relative = summary["rms_change_percent"]
        change_text = f"RMS change {relative:.3f}%" if np.isfinite(relative) else "Capture baseline to see changes"
        self.description = f"{source} · {summary['frames']} frames · {change_text} · clipped {count}/{channels}"
        if zeros:
            indices = ", ".join(map(str, np.flatnonzero(zero_channels)[:8] + 1))
            self.description += f" · zero {zeros}/{channels}: channels {indices}" + ("…" if zeros > 8 else "")
        self.warning_line.set_ydata([self.warning_percent,self.warning_percent])
        valid_usage = summary["usage_percent"][np.isfinite(summary["usage_percent"])]
        self.axes[3].set_ylim(-5,max(115,float(valid_usage.max())*1.1 if len(valid_usage) else 115))
        if len(valid_usage):
            self.description += f" · peak clean sine-range use {valid_usage.max():.1f}% (estimate)"
            if valid_usage.max() >= self.warning_percent:self.description += " · near/above range limit"
        else:
            self.description += " · range unavailable (missing gains/demo, or all channels clipped)"
        if count:self.description += " · clipped amplitudes cannot indicate headroom"
        discarded = capture.get("discarded_invalid_frames", 0)
        if discarded:
            self.description += f" · discarded incomplete/invalid frames: {discarded}"
        self.figure.canvas.draw_idle()

    def unavailable(self, message):
        # Clear old channel readings so a failed acquisition never looks live.
        for line in (self.magnitude, self.phase, self.real, self.imaginary, self.clipped, self.zeros, self.usage, self.usage_clipped, self.usage_zero):
            line.set_data([], [])
        self.summary = None
        self.last_capture = None
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


class SignalChannelTable:
    """Per-configuration voltage, gain and flags; never changes saved raw values."""
    def __init__(self, master):
        from tkinter import ttk
        self.device = {}
        self.last_summary = None
        columns = ('channel','drive','sense','gain','voltage','ceiling','usage','clip','zero','variation','flags')
        self.tree = ttk.Treeview(master, columns=columns, show='headings', height=14)
        titles = ('Ch','TX + / −','RX + / −','RX gain','Mean mV','Limit mV*','Peak %*','Clip %','Zero %','Variation %','Flags')
        for name,title in zip(columns,titles):
            self.tree.heading(name,text=title)
            self.tree.column(name,width=90 if name!='flags' else 260,minwidth=55,stretch=False)
        scroll = ttk.Scrollbar(master, orient='horizontal', command=self.tree.xview)
        vertical = ttk.Scrollbar(master, orient='vertical', command=self.tree.yview)
        self.tree.configure(xscrollcommand=scroll.set,yscrollcommand=vertical.set)
        vertical.pack(side='right',fill='y');scroll.pack(side='bottom',fill='x')
        self.tree.pack(fill='both',expand=True)
        self.tree.tag_configure('fault',foreground='#B33B00')
        self.tree.tag_configure('warning',foreground='#755300')

    def update(self, summary, device=None, warning_percent=80., low_percent=5.):
        if summary is self.last_summary:return
        self.last_summary=summary
        self.tree.delete(*self.tree.get_children())
        if summary is None:return
        protocol=(device or {}).get('configuration_order_txp_txn_rxp_rxn',[])
        flags=quality_flags(summary,warning_percent,low_percent)
        def number(value):return f'{value:.3g}' if np.isfinite(value) else '—'
        for i,flag in enumerate(flags):
            pins=protocol[i] if i<len(protocol) else None
            tx=f'{pins[0]} / {pins[1]}' if pins else '—'
            rx=f'{pins[2]} / {pins[3]}' if pins else '—'
            values=(i+1,tx,rx,number(summary['rx_gain'][i]),number(summary['magnitude_mV'][i]),
                    number(summary['limit_mV'][i]),number(summary['usage_percent'][i]),
                    number(100*summary['clipped_fraction'][i]),number(100*summary['zero_fraction'][i]),
                    number(summary['frame_noise_percent'][i]),flag)
            tag='fault' if summary['clipped'][i] or summary['zero_channels'][i] else 'warning' if flag!='no flag' else ''
            self.tree.insert('', 'end', values=values, tags=(tag,))


def save_signal_snapshot(path, plot):
    """Save raw capture, baseline, gain metadata and diagnostics without NaN JSON."""
    import json
    from pathlib import Path
    from datetime import datetime, timezone
    if plot.last_capture is None:raise ValueError('No current valid capture to save')
    def plain(value):
        if isinstance(value, np.ndarray):return plain(value.tolist())
        if isinstance(value, np.generic):return plain(value.item())
        if isinstance(value, complex):return {'real':plain(value.real),'imag':plain(value.imag)}
        if isinstance(value, float) and not np.isfinite(value):return None
        if isinstance(value, dict):return {k:plain(v) for k,v in value.items()}
        if isinstance(value, (list,tuple)):return [plain(v) for v in value]
        return value
    report={'created_utc':datetime.now(timezone.utc).isoformat(),'device':plot.device,
            'adc_peak_limit_v_estimate':plot.adc_peak_limit_v,'description':plot.description,
            'capture':plot.last_capture,'baseline':plot.last_baseline,'summary':plot.summary,
            'flags':quality_flags(plot.summary),
            'range_note':'Sine-equivalent estimate from supplied v1.1.1 source; not measured waveform peak. Clipped amplitudes do not indicate headroom.'}
    Path(path).write_text(json.dumps(plain(report),indent=2,allow_nan=False)+'\n')
