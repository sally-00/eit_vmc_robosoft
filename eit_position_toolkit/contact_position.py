"""Position-only EIT learning, storage and numbered STL view (no robot control)."""
from __future__ import annotations

import json
from pathlib import Path
from datetime import datetime, timezone
from zoneinfo import ZoneInfo
import uuid

import numpy as np
from eit_reader import measurement_protocol
from contact_mlp import ContactMLP

HERE = Path(__file__).resolve().parent
PLATES = HERE / "assets"
MODEL_VERSION = 1
MLP_MODEL_VERSION = 2


def targets(count=20, radius_mm=45., margin_mm=5.):
    """Centre plus equal-area sunflower samples, inset by the requested margin."""
    if type(count) is not int or not 3 <= count <= 500:
        raise ValueError("Choose 3–500 training points")
    if not np.isfinite([radius_mm, margin_mm]).all() or not 0 <= margin_mm < radius_mm:
        raise ValueError("Radius must be positive and margin must be smaller than radius")
    i = np.arange(1, count)
    r = (radius_mm - margin_mm) * np.sqrt(i / (count - 1))
    theta = i * np.pi * (3 - np.sqrt(5))
    return np.vstack(([0., 0.], np.column_stack((r * np.cos(theta), r * np.sin(theta)))))


def voltages(capture, allow_clipped=False):
    frames = capture["frames"]
    if not frames:
        raise ValueError("Empty EIT capture; no measurement frames received")
    if not allow_clipped and any(any(f["clipped"]) for f in frames):
        raise ValueError("Clipped EIT capture; enable Allow clipped readings to continue, or check gain")
    mag = np.asarray([f["magnitude_Vrms"] for f in frames], float)
    phase = np.asarray([f["phase_rad"] for f in frames], float)
    clipped = np.asarray([f["clipped"] for f in frames], bool)
    if mag.ndim != 2 or mag.shape[1] == 0 or phase.shape != mag.shape or clipped.shape != mag.shape:
        raise ValueError("Expected matching magnitude, phase and clipping arrays per frame")
    if not np.isfinite(mag).all() or not np.isfinite(phase).all() or np.any(mag < 0):
        raise ValueError("Invalid EIT voltage: non-finite or negative measurement")
    if not np.any(mag > 1e-12):
        raise ValueError("All EIT voltage channels are zero; check electrode connections and excitation")
    return mag * np.exp(1j * phase)


def baseline_reference(voltage, channel_mask=None):
    """Never divide by a zero baseline; a supplied model mask stays fixed."""
    base = voltage.mean(axis=0)
    available = np.abs(base) > 1e-12
    mask = available if channel_mask is None else np.asarray(channel_mask, dtype=bool)
    if mask.shape != available.shape:
        raise ValueError("Baseline channel count does not match the model")
    if not mask.any():
        raise ValueError("No usable baseline voltage channels; check electrode connections")
    missing = mask & ~available
    if missing.any():
        indices = ", ".join(map(str, np.flatnonzero(missing)[:8] + 1))
        raise ValueError(f"Test baseline is zero in required model channels {indices}; recapture baseline or retrain")
    return base, mask


def signature(capture, baseline, allow_clipped=False, channel_mask=None):
    base, mask = baseline_reference(voltages(baseline, allow_clipped), channel_mask)
    current = voltages(capture, allow_clipped).mean(axis=0)
    if current.shape != base.shape:
        raise ValueError("Contact and baseline channel counts differ; capture a baseline for the current scan")
    delta = np.zeros_like(base)
    delta[mask] = (current[mask] - base[mask]) / base[mask]
    vector = np.r_[delta.real, delta.imag]
    strength = float(np.linalg.norm(vector))
    return vector / max(strength, 1e-12), strength


class PositionModel:
    """RBF or MLP regression on unit complex-voltage changes; no force labels."""
    def __init__(self, x, xy, threshold, metadata, mlp_parameters=None):
        self.x = np.asarray(x, float)
        self.xy = np.asarray(xy, float)
        self.threshold = float(threshold)
        self.metadata = metadata
        self.model_type = metadata.setdefault("model_type", "rbf")
        if self.model_type not in ("rbf", "mlp"):
            raise ValueError("Choose rbf or mlp as the training model")
        self.metadata.setdefault("allow_clipped", False)
        self.metadata["contact_strength_check"] = False
        self.metadata["unfamiliar_predictions"] = True
        if (self.x.ndim != 2 or self.x.shape[1] < 2 or self.x.shape[1] % 2 or len(self.x) < 3
                or self.xy.shape != (len(self.x), 2)
                or not np.isfinite(self.x).all() or not np.isfinite(self.xy).all()
                or not np.isfinite(self.threshold) or self.threshold < 0):
            raise ValueError("Invalid contact-position model arrays")
        channels = self.x.shape[1] // 2
        settings = metadata.get("eit_config", {})
        pins = settings.get("eit_electrode_pins")
        drive_pattern = settings.get("eit_drive_pattern", "adjacent")
        if pins is not None and channels != len(measurement_protocol(pins, drive_pattern)):
            raise ValueError("Model input size does not match its saved electrode scan")
        self.metadata["drive_pattern"] = drive_pattern
        self.metadata["measurement_channels"] = channels
        self.metadata["input_features"] = self.x.shape[1]
        self.channel_mask = np.asarray(metadata.get("usable_channel_mask", [True]*channels), dtype=bool)
        if self.channel_mask.shape != (channels,) or not self.channel_mask.any():
            raise ValueError("Invalid model channel selection")
        self.metadata["usable_channel_mask"] = self.channel_mask.tolist()
        self.metadata["usable_measurement_channels"] = int(self.channel_mask.sum())
        distance = self.distances(self.x, self.x)
        positive = distance[distance > .02]
        self.width = max(.15, float(np.median(positive)) if len(positive) else .5)
        if self.model_type == "rbf":
            kernel = np.exp(-distance ** 2 / (2 * self.width ** 2))
            self.weights = np.linalg.solve(kernel + .005 * np.eye(len(self.x)), self.xy)
        else:
            self.mlp = ContactMLP(self.x.shape[1], metadata["radius_mm"], mlp_parameters)
            if mlp_parameters is None:
                self.metadata["mlp_training"] = self.mlp.fit(self.x, self.xy)
        # Flag unfamiliar predictions without suppressing their coordinates.
        np.fill_diagonal(distance, np.inf)
        self.support_limit = min(1., max(.25, float(np.quantile(distance.min(axis=1), .95)) * 3))

    @staticmethod
    def distances(a, b):
        return np.sqrt(np.maximum(0, np.sum(a*a, axis=1)[:, None]
                                  + np.sum(b*b, axis=1)[None, :] - 2 * a @ b.T))

    def estimate(self, x):
        d = self.distances(np.atleast_2d(x), self.x)
        xy = (self.mlp.predict(x) if self.model_type == "mlp" else
              np.exp(-d ** 2 / (2 * self.width ** 2)) @ self.weights)
        return xy, d.min(axis=1)

    def predict(self, capture, baseline, allow_clipped=None):
        if allow_clipped is None:
            allow_clipped = self.metadata["allow_clipped"]
        if voltages(capture, allow_clipped).shape[1] != len(self.channel_mask):
            raise ValueError(f"Model expects {len(self.channel_mask)} measurement channels; retrain for this electrode setup")
        x, strength = signature(capture, baseline, allow_clipped, self.channel_mask)
        if len(x) != self.x.shape[1]:
            raise ValueError(f"Model expects {self.x.shape[1]//2} measurement channels; retrain for this electrode setup")
        xy, distance = self.estimate(x)
        radius = self.metadata["radius_mm"]
        pattern_distance = float(distance[0])
        pattern_familiar = pattern_distance <= self.support_limit
        within_pad = bool(np.linalg.norm(xy[0]) <= radius)
        unfamiliar = not pattern_familiar or not within_pad
        return {"state": "unfamiliar" if unfamiliar else "contact",
                "xy_mm": xy[0].tolist(), "strength": strength,
                "strength_rms_percent": float(100 * strength / np.sqrt(self.channel_mask.sum())),
                "pattern_distance": pattern_distance, "pattern_limit": self.support_limit,
                "pattern_familiar": pattern_familiar, "within_pad": within_pad}

    def save(self, path):
        parameters = self.mlp.arrays() if self.model_type == "mlp" else {}
        version = MLP_MODEL_VERSION if self.model_type == "mlp" else MODEL_VERSION
        np.savez_compressed(path, x=self.x, xy=self.xy, threshold=self.threshold,
                            metadata=json.dumps({**self.metadata, "model_version": version}), **parameters)

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            metadata = json.loads(str(data["metadata"]))
            version = MLP_MODEL_VERSION if metadata.get("model_type") == "mlp" else MODEL_VERSION
            if metadata.get("model_version") != version:
                raise ValueError("Unsupported model version")
            if metadata.get("numbering") not in ("ccw", "cw"):
                raise ValueError("Model is missing its plate orientation")
            if not np.isfinite(metadata["radius_mm"]) or metadata["radius_mm"] <= 0:
                raise ValueError("Invalid model radius")
            parameters = data if metadata.get("model_type", "rbf") == "mlp" else None
            return cls(data["x"], data["xy"], data["threshold"], metadata, parameters)


def fit_samples(records, metadata):
    if len({r["point_id"] for r in records}) < 3:
        raise ValueError("Collect at least three distinct locations")
    vectors = []
    allow_clipped = bool(metadata.get("allow_clipped", False))
    masks = [np.abs(voltages(record["baseline"], allow_clipped).mean(axis=0)) > 1e-12
             for record in records]
    if len({len(mask) for mask in masks}) != 1:
        raise ValueError("Training baseline channel counts differ")
    channel_mask = np.logical_and.reduce(masks)
    if not channel_mask.any():
        raise ValueError("No common usable baseline channels across training touches; check electrode connections")
    metadata["usable_channel_mask"] = channel_mask.tolist()
    for record in records:
        x, _ = signature(record["contact"], record["baseline"], allow_clipped, channel_mask)
        vectors.append(x)
    # Keep the legacy NPZ field so existing model files remain readable.
    # The position app no longer uses a contact-strength threshold.
    threshold = 0.
    xy = [r["xy_mm"] for r in records]
    model = PositionModel(vectors, xy, threshold, metadata)
    # Entire independent touch repetitions are held out; frames from one hold stay together.
    train = np.array([r["repeat"] != max(t["repeat"] for t in records) for r in records])
    report = {"validation": "unavailable: choose at least two repetitions"}
    if train.any() and (~train).any():
        check = PositionModel(np.array(vectors)[train], np.array(xy)[train], threshold, dict(metadata))
        predicted, _ = check.estimate(np.array(vectors)[~train])
        errors = np.linalg.norm(predicted - np.array(xy)[~train], axis=1)
        report = {"validation": "last touch repetition held out at each trained point",
                  "held_out_touches": int(len(errors)), "median_error_mm": float(np.median(errors)),
                  "max_error_mm": float(errors.max()),
                  "limitation": "Same locations; does not measure error at unseen positions or in later sessions"}
    report["allow_clipped"] = allow_clipped
    report["model_type"] = model.model_type
    if model.model_type == "mlp":
        report["mlp_training"] = model.metadata["mlp_training"]
    report["contact_strength_check"] = False
    report["unfamiliar_predictions"] = True
    report["usable_measurement_channels"] = int(channel_mask.sum())
    report["excluded_channels"] = (np.flatnonzero(~channel_mask) + 1).tolist()
    report["clipped_frames"] = sum(any(frame["clipped"]) for record in records
                                    for kind in ("baseline", "contact") for frame in record[kind]["frames"])
    model.metadata["evaluation"] = report
    return model, report


def new_session(root, metadata):
    started = datetime.now(ZoneInfo("Asia/Tokyo"))
    stamp = started.strftime("%Y-%m-%d_%H-%M-%S_JST")
    metadata["created_local"] = started.isoformat()
    metadata["session_timezone"] = "Asia/Tokyo"
    path = Path(root) / (stamp + "_" + uuid.uuid4().hex[:8])
    path.mkdir(parents=True, exist_ok=False)
    (path / "manifest.json").write_text(json.dumps(metadata, indent=2))
    return path


def plate_triangles(numbering):
    """Read the actual binary STL; keep upward faces, including engraved floors."""
    if numbering == "ccw":
        path = PLATES / "numbered_retaining_plate_ccw/Frame_base_3_numbered_CCW_1-8_engraved_mm.stl"
    elif numbering == "cw":
        path = PLATES / "numbered_retaining_plate/Frame_base_3_numbered_1-8_engraved_mm.stl"
    else:
        raise ValueError("Numbering must be ccw or cw")
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    with path.open("rb") as stream:
        stream.seek(80)
        count = int.from_bytes(stream.read(4), "little")
        triangles = np.fromfile(stream, dtype=dtype, count=count)
    if len(triangles) != count:
        raise ValueError("Truncated plate STL")
    top = triangles[triangles["normal"][:, 2] > .01]["vertices"]
    return top


def draw_pad(ax, numbering="ccw", radius_mm=45.):
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Circle
    top = plate_triangles(numbering)
    top = top[np.argsort(top[:, :, 2].mean(axis=1))]
    z = top[:, :, 2].mean(axis=1)
    color = np.tile([.31, .61, .73, 1.], (len(z), 1))
    color[:, :3] *= (.42 + .58 * np.clip((z - 2.49) / .51, 0, 1))[:, None]
    ax.add_collection(PolyCollection(top[:, :, :2], facecolors=color, edgecolors="none",
                                     antialiaseds=False, rasterized=True))
    ax.add_patch(Circle((0, 0), radius_mm, facecolor="#edf5f6", edgecolor="#526c76", lw=1))
    ax.axhline(0, color="#ccdce0", lw=.6, zorder=0)
    ax.axvline(0, color="#ccdce0", lw=.6, zorder=0)
    # Upright callouts supplement the radially engraved digits in the mesh.
    for i in range(8):
        angle = np.deg2rad(-90 + i * 45 if numbering == "ccw" else 90 - i * 45)
        ax.text(70 * np.cos(angle), 70 * np.sin(angle), str(i + 1),
                ha="center", va="center", fontsize=12, weight="bold", color="#234955")
    ax.set(xlim=(-77, 77), ylim=(-77, 77), aspect="equal", xlabel="Pad X (mm)", ylabel="Pad Y (mm)")
    ax.set_title("Engraved face · " + ("counterclockwise" if numbering == "ccw" else "clockwise"))
    return ax


class DemoReader:
    """Synthetic UI exercise only. It is deliberately not a physical EIT simulator."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.drive_pattern = cfg.get("eit_drive_pattern", "adjacent")
        self.protocol = measurement_protocol(cfg["eit_electrode_pins"], self.drive_pattern)
        self.channels = len(self.protocol)
        self.centers = targets(self.channels, 55, 0)
        self.xy = None
        self.rng = np.random.default_rng(21)

    def open(self):
        pass

    def close(self):
        pass

    def metadata(self):
        return {"demo": True, "description": "Synthetic UI signals; no hardware accuracy claim",
                "electrode_count": len(self.cfg["eit_electrode_pins"]),
                "measurement_channels": self.channels,
                "drive_pattern": self.drive_pattern,
                "rx_gain_margin_steps": self.cfg.get("eit_rx_gain_margin_steps", 0),
                "rx_gain_margin_note": "Synthetic signals; no physical amplifier gain",
                "configuration_order_txp_txn_rxp_rxn": self.protocol}

    def capture(self):
        import time
        time.sleep(.08)
        xy = None if self.xy is None else np.array(self.xy)
        response = np.zeros(self.channels) if xy is None else .035 * np.exp(-np.sum((self.centers-xy)**2, axis=1)/800)
        frames = []
        for _ in range(self.cfg.get("eit_frames_per_capture", 5)):
            magnitude = .01 * (1 + response + self.rng.normal(0, .00003, self.channels))
            frames.append({"magnitude_Vrms": magnitude.tolist(), "phase_rad": [0.] * self.channels,
                           "clipped": [False] * self.channels, "host_utc": datetime.now(timezone.utc).isoformat()})
        return {"frames": frames, "frame_count": len(frames)}
