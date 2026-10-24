"""Read-only EIT position decoder: load exported parameters and predict."""
from __future__ import annotations
import json
import numpy as np
MODEL_VERSION = 1
INFERENCE_FORMAT_VERSION = 1

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
    """RBF inference with precomputed weights; no fitting or model export."""
    def __init__(self, x, weights, width, support_limit, threshold, metadata):
        self.x = np.asarray(x, float)
        self.weights = np.asarray(weights, float)
        self.width = float(width)
        self.support_limit = float(support_limit)
        self.threshold = float(threshold)
        self.metadata = metadata
        if (self.x.ndim != 2 or self.x.shape[1] != 80 or len(self.x) < 3
                or self.weights.shape != (len(self.x), 2)
                or not np.isfinite(self.x).all() or not np.isfinite(self.weights).all()
                or not np.isfinite([self.width, self.support_limit, self.threshold]).all()
                or self.width <= 0 or self.support_limit <= 0 or self.threshold < 0):
            raise ValueError("Invalid exported contact-position model parameters")

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

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            metadata = json.loads(str(data["metadata"]))
            if metadata.get("model_version") != MODEL_VERSION or metadata.get("model_type", "rbf") != "rbf":
                raise ValueError("Unsupported model: expected a 40-channel RBF decoder")
            required = {"x", "weights", "width", "support_limit", "threshold"}
            if (metadata.get("inference_format_version") != INFERENCE_FORMAT_VERSION
                    or not required.issubset(data.files)):
                raise ValueError("Model lacks precomputed inference parameters; run export_franka_model.py on the model-preparation computer")
            if metadata.get("numbering") not in ("ccw", "cw"):
                raise ValueError("Model is missing its plate orientation")
            if not np.isfinite(metadata["radius_mm"]) or metadata["radius_mm"] <= 0:
                raise ValueError("Invalid model radius")
            mask = np.asarray(metadata.get("usable_channel_mask", [True] * 40), dtype=bool)
            if (mask.shape != (40,) or not mask.all()
                    or metadata.get("eit_config", {}).get("eit_drive_pattern", "adjacent") != "adjacent"):
                raise ValueError("Expected all 40 adjacent-scan channels")
            return cls(data["x"], data["weights"], data["width"], data["support_limit"], data["threshold"], metadata)
