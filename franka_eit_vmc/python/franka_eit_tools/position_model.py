"""Read the trained EIT checkpoint directly for position inference."""
from __future__ import annotations

import json
import numpy as np
from .eit_reader import measurement_protocol


def voltages(capture, channels, allow_clipped=False):
    frames = capture["frames"]
    if not frames:
        raise ValueError("Empty EIT capture")
    mag = np.asarray([f["magnitude_Vrms"] for f in frames], float)
    phase = np.asarray([f["phase_rad"] for f in frames], float)
    clipped = np.asarray([f["clipped"] for f in frames], bool)
    if mag.shape != (len(frames), channels) or phase.shape != mag.shape or clipped.shape != mag.shape:
        raise ValueError(f"Expected {channels} magnitude, phase and clipping fields per frame")
    if clipped.any() and not allow_clipped:
        raise ValueError("Clipped EIT frame")
    if not np.isfinite(mag).all() or not np.isfinite(phase).all() or (mag < 0).any():
        raise ValueError("Invalid EIT voltage")
    if not (mag > 1e-12).any():
        raise ValueError("All EIT channels are zero")
    return mag * np.exp(1j * phase)


def baseline_reference(voltage, mask):
    base = voltage.mean(axis=0)
    if (np.abs(base[mask]) <= 1e-12).any():
        raise ValueError("Zero baseline in a required model channel")
    return base


def signature(capture, baseline, allow_clipped=False, channel_mask=None):
    if not capture.get("frames"):
        raise ValueError("Empty EIT capture")
    channels = len(capture["frames"][0]["magnitude_Vrms"])
    mask = np.ones(channels, bool) if channel_mask is None else np.asarray(channel_mask, bool)
    if mask.shape != (channels,):
        raise ValueError("Model and capture channel counts differ")
    base = baseline_reference(voltages(baseline, channels, allow_clipped), mask)
    current = voltages(capture, channels, allow_clipped).mean(axis=0)
    delta = np.zeros(channels, complex)
    delta[mask] = (current[mask] - base[mask]) / base[mask]
    vector = np.r_[delta.real, delta.imag]
    strength = float(np.linalg.norm(vector))
    return vector / max(strength, 1e-12), strength


def noise_level(baseline, allow_clipped=False, channel_mask=None):
    if not baseline.get("frames"):
        raise ValueError("Empty EIT baseline")
    channels = len(baseline["frames"][0]["magnitude_Vrms"])
    mask = np.ones(channels, bool) if channel_mask is None else np.asarray(channel_mask, bool)
    if mask.shape != (channels,):
        raise ValueError("Model and baseline channel counts differ")
    voltage = voltages(baseline, channels, allow_clipped)
    if len(voltage) < 5:
        raise ValueError("Baseline needs at least five frames")
    base = baseline_reference(voltage, mask)
    delta = (voltage[:, mask] - base[mask]) / base[mask]
    return max(1e-5, float(np.quantile(np.linalg.norm(delta, axis=1), .95)))


class PositionModel:
    """RBF model calculated from saved training vectors and labels."""

    def __init__(self, x, xy, threshold, metadata):
        self.x = np.asarray(x, float)
        self.xy = np.asarray(xy, float)
        self.threshold = float(threshold)
        self.metadata = metadata
        if self.x.ndim != 2 or self.x.shape[1] < 2 or self.x.shape[1] % 2:
            raise ValueError("Invalid model feature shape")
        self.channels = self.x.shape[1] // 2
        settings = metadata.get("eit_config", {})
        self.protocol = measurement_protocol(settings.get("eit_electrode_pins"),
                                             settings.get("eit_drive_pattern", "adjacent"))
        self.channel_mask = np.asarray(metadata.get("usable_channel_mask", [True] * self.channels), bool)
        if (self.channels != len(self.protocol) or self.channel_mask.shape != (self.channels,)
                or not self.channel_mask.any() or len(self.x) < 3
                or self.xy.shape != (len(self.x), 2)
                or not np.isfinite(self.x).all() or not np.isfinite(self.xy).all()
                or not np.isfinite(self.threshold) or self.threshold < 0):
            raise ValueError("Invalid model arrays or saved scan configuration")
        self.allow_clipped = metadata.get("allow_clipped", False)
        if type(self.allow_clipped) is not bool:
            raise ValueError("allow_clipped must be a boolean")
        self.radius = float(metadata["radius_mm"])
        if not np.isfinite(self.radius) or self.radius <= 0:
            raise ValueError("Invalid pad radius")
        distance = self.distances(self.x, self.x)
        positive = distance[distance > .02]
        self.width = max(.15, float(np.median(positive)) if len(positive) else .5)
        kernel = np.exp(-distance ** 2 / (2 * self.width ** 2))
        self.weights = np.linalg.solve(kernel + .005 * np.eye(len(self.x)), self.xy)
        np.fill_diagonal(distance, np.inf)
        self.support_limit = min(1., max(.25, float(np.quantile(distance.min(axis=1), .95)) * 3))

    @staticmethod
    def distances(a, b):
        return np.sqrt(np.maximum(0, np.sum(a*a, axis=1)[:, None]
                                  + np.sum(b*b, axis=1)[None, :] - 2 * a @ b.T))

    def estimate(self, features):
        distance = self.distances(np.atleast_2d(features), self.x)
        return np.exp(-distance ** 2 / (2 * self.width ** 2)) @ self.weights, distance.min(axis=1)

    def signature(self, capture, baseline):
        return signature(capture, baseline, self.allow_clipped, self.channel_mask)

    def noise_level(self, baseline):
        return noise_level(baseline, self.allow_clipped, self.channel_mask)

    def predict(self, capture, baseline, contact_threshold=0.0):
        features, strength = self.signature(capture, baseline)
        xy, distance = self.estimate(features)
        threshold = float(contact_threshold)
        if not np.isfinite(threshold) or threshold < 0:
            raise ValueError('Contact threshold must be finite and nonnegative')
        familiar = (strength > 1e-12 and strength >= threshold and
                    distance[0] <= self.support_limit
                    and np.linalg.norm(xy[0]) <= self.radius)
        return {"state": "contact" if familiar else "unfamiliar", "xy_mm": xy[0].tolist(),
                "strength": strength, "pattern_distance": float(distance[0])}

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            metadata = json.loads(str(data["metadata"]))
            if metadata.get("model_version") != 1 or metadata.get("model_type", "rbf") != "rbf":
                raise ValueError("Unsupported model type or version; expected trained RBF checkpoint")
            if metadata.get("numbering") not in ("ccw", "cw"):
                raise ValueError("Model is missing its plate orientation")
            return cls(data["x"], data["xy"], data["threshold"], metadata)
