"""Portable physical-pad EIT model, derived from the existing contact-position collector."""
from __future__ import annotations
import json
from pathlib import Path
from datetime import datetime, timezone
import uuid
import numpy as np
MODEL_VERSION = 1

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


def voltages(capture):
    frames = capture["frames"]
    if not frames or any(any(f["clipped"]) for f in frames):
        raise ValueError("Clipped or empty EIT capture; release and retry after checking gain")
    mag = np.asarray([f["magnitude_Vrms"] for f in frames], float)
    phase = np.asarray([f["phase_rad"] for f in frames], float)
    if mag.ndim != 2 or mag.shape[1] != 40 or phase.shape != mag.shape:
        raise ValueError("Expected 40 magnitude and phase measurements per frame")
    if not np.isfinite(mag).all() or not np.isfinite(phase).all() or np.any(mag <= 1e-12):
        raise ValueError("Invalid/zero EIT voltage; check electrode connections")
    return mag * np.exp(1j * phase)


def signature(capture, baseline):
    base = voltages(baseline).mean(axis=0)
    delta = (voltages(capture).mean(axis=0) - base) / base
    vector = np.r_[delta.real, delta.imag]
    strength = float(np.linalg.norm(vector))
    return vector / max(strength, 1e-12), strength


def noise_level(baseline):
    v = voltages(baseline)
    if len(v) < 5:
        raise ValueError("Baseline needs at least five frames")
    delta = (v - v.mean(axis=0)) / v.mean(axis=0)
    return max(1e-5, float(np.quantile(np.linalg.norm(delta, axis=1), .95)))


class PositionModel:
    """RBF kernel ridge on unit complex-voltage changes; no force labels."""
    def __init__(self, x, xy, threshold, metadata):
        self.x = np.asarray(x, float)
        self.xy = np.asarray(xy, float)
        self.threshold = float(threshold)
        self.metadata = metadata
        if (self.x.ndim != 2 or self.x.shape[1] != 80 or len(self.x) < 3
                or self.xy.shape != (len(self.x), 2)
                or not np.isfinite(self.x).all() or not np.isfinite(self.xy).all()
                or not np.isfinite(self.threshold) or self.threshold <= 0):
            raise ValueError("Invalid contact-position model arrays")
        distance = self.distances(self.x, self.x)
        positive = distance[distance > .02]
        self.width = max(.15, float(np.median(positive)) if len(positive) else .5)
        kernel = np.exp(-distance ** 2 / (2 * self.width ** 2))
        self.weights = np.linalg.solve(kernel + .005 * np.eye(len(self.x)), self.xy)
        # A novelty gate, not a probability/confidence score.
        np.fill_diagonal(distance, np.inf)
        self.support_limit = min(1., max(.25, float(np.quantile(distance.min(axis=1), .95)) * 3))

    @staticmethod
    def distances(a, b):
        return np.sqrt(np.maximum(0, np.sum(a*a, axis=1)[:, None]
                                  + np.sum(b*b, axis=1)[None, :] - 2 * a @ b.T))

    def estimate(self, x):
        d = self.distances(np.atleast_2d(x), self.x)
        return np.exp(-d ** 2 / (2 * self.width ** 2)) @ self.weights, d.min(axis=1)

    def predict(self, capture, baseline):
        x, strength = signature(capture, baseline)
        threshold = max(self.threshold, 5 * noise_level(baseline))
        if strength < threshold:
            return {"state": "no_contact", "xy_mm": None, "strength": strength}
        xy, distance = self.estimate(x)
        radius = self.metadata["radius_mm"]
        if distance[0] > self.support_limit or np.linalg.norm(xy[0]) > radius:
            return {"state": "unfamiliar", "xy_mm": None, "strength": strength}
        return {"state": "contact", "xy_mm": xy[0].tolist(), "strength": strength}

    def save(self, path):
        np.savez_compressed(path, x=self.x, xy=self.xy, threshold=self.threshold,
                            metadata=json.dumps({**self.metadata, "model_version": MODEL_VERSION}))

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            metadata = json.loads(str(data["metadata"]))
            if metadata.get("model_version") != MODEL_VERSION:
                raise ValueError("Unsupported model version")
            if metadata.get("numbering") not in ("ccw", "cw"):
                raise ValueError("Model is missing its plate orientation")
            if not np.isfinite(metadata["radius_mm"]) or metadata["radius_mm"] <= 0:
                raise ValueError("Invalid model radius")
            return cls(data["x"], data["xy"], data["threshold"], metadata)


def fit_samples(records, metadata):
    if len({r["point_id"] for r in records}) < 3:
        raise ValueError("Collect at least three distinct locations")
    vectors, strengths, noise = [], [], []
    for record in records:
        x, strength = signature(record["contact"], record["baseline"])
        n = noise_level(record["baseline"])
        if strength < 8 * n:
            raise ValueError(f"Point {record['point_id'] + 1} has too little signal; recapture it")
        vectors.append(x)
        strengths.append(strength)
        noise.append(n)
    threshold = max(5 * max(noise), .2 * min(strengths))
    if threshold >= min(strengths):
        raise ValueError("Baseline noise overlaps touch signal; collect again with stable connections")
    xy = [r["xy_mm"] for r in records]
    model = PositionModel(vectors, xy, threshold, metadata)
    # Entire independent touch repetitions are held out; frames from one hold stay together.
    train = np.array([r["repeat"] != max(t["repeat"] for t in records) for r in records])
    report = {"validation": "unavailable: choose at least two repetitions"}
    if train.any() and (~train).any():
        check = PositionModel(np.array(vectors)[train], np.array(xy)[train], threshold, metadata)
        predicted, _ = check.estimate(np.array(vectors)[~train])
        errors = np.linalg.norm(predicted - np.array(xy)[~train], axis=1)
        report = {"validation": "last touch repetition held out at each trained point",
                  "held_out_touches": int(len(errors)), "median_error_mm": float(np.median(errors)),
                  "max_error_mm": float(errors.max()),
                  "limitation": "Same locations; does not measure error at unseen positions or in later sessions"}
    model.metadata["evaluation"] = report
    return model, report


def new_session(root, metadata):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    path = Path(root) / (stamp + "_" + uuid.uuid4().hex[:8])
    path.mkdir(parents=True, exist_ok=False)
    (path / "manifest.json").write_text(json.dumps(metadata, indent=2))
    return path



class DemoReader:
    """Synthetic UI exercise only. It is deliberately not a physical EIT simulator."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.xy = None
        self.rng = np.random.default_rng(21)

    def open(self):
        pass

    def close(self):
        pass

    def metadata(self):
        return {"demo": True, "description": "Synthetic UI signals; no hardware accuracy claim"}

    def capture(self):
        import time
        time.sleep(.08)
        centers = targets(40, 55, 0)
        xy = None if self.xy is None else np.array(self.xy)
        response = np.zeros(40) if xy is None else .035 * np.exp(-np.sum((centers-xy)**2, axis=1)/800)
        frames = []
        for _ in range(self.cfg.get("eit_frames_per_capture", 5)):
            magnitude = .01 * (1 + response + self.rng.normal(0, .00003, 40))
            frames.append({"magnitude_Vrms": magnitude.tolist(), "phase_rad": [0.] * 40,
                           "clipped": [False] * 40, "host_utc": datetime.now(timezone.utc).isoformat()})
        return {"frames": frames, "frame_count": len(frames)}
