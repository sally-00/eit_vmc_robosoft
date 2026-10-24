#!/usr/bin/env python3
"""Export an existing 40-channel RBF checkpoint for prediction-only Franka deployment.

Run on the model-preparation computer, outside the ROS package:
  python3 export_franka_model.py --input position_model.npz --output deploy_model.npz
The original checkpoint is left unchanged. No touch data are collected here.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def export_model(source, destination):
    source, destination = Path(source), Path(destination)
    if source.resolve() == destination.resolve():
        raise ValueError("Choose a separate output; keep the original checkpoint")
    with np.load(source, allow_pickle=False) as data:
        metadata = json.loads(str(data["metadata"]))
        if metadata.get("model_version") != 1 or metadata.get("model_type", "rbf") != "rbf":
            raise ValueError("Franka currently supports 40-channel RBF checkpoints only")
        x, xy = np.asarray(data["x"], float), np.asarray(data["xy"], float)
        threshold = float(data["threshold"])
    if (x.ndim != 2 or x.shape[1] != 80 or len(x) < 3 or xy.shape != (len(x), 2)
            or not np.isfinite(x).all() or not np.isfinite(xy).all()
            or not np.isfinite(threshold) or threshold < 0):
        raise ValueError("Expected finite 80-feature RBF arrays and a nonnegative threshold")
    mask = np.asarray(metadata.get("usable_channel_mask", [True] * 40), dtype=bool)
    settings = metadata.get("eit_config", {})
    if (mask.shape != (40,) or not mask.all()
            or settings.get("eit_drive_pattern", "adjacent") != "adjacent"):
        raise ValueError("Franka currently requires all 40 adjacent-scan channels")
    radius = metadata.get("radius_mm", 0)
    if metadata.get("numbering") not in ("ccw", "cw") or not np.isfinite(radius) or radius <= 0:
        raise ValueError("Checkpoint must specify plate numbering and a positive radius_mm")
    distance = np.sqrt(np.maximum(0, np.sum(x*x, axis=1)[:, None]
                                  + np.sum(x*x, axis=1)[None, :] - 2 * x @ x.T))
    positive = distance[distance > .02]
    width = max(.15, float(np.median(positive)) if len(positive) else .5)
    kernel = np.exp(-distance ** 2 / (2 * width ** 2))
    weights = np.linalg.solve(kernel + .005 * np.eye(len(x)), xy)
    np.fill_diagonal(distance, np.inf)
    support_limit = min(1., max(.25, float(np.quantile(distance.min(axis=1), .95)) * 3))
    metadata.update(model_type="rbf", inference_format_version=1)
    if destination.suffix != ".npz":
        raise ValueError("Output must end in .npz")
    destination.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(destination, x=x, xy=xy, threshold=threshold, weights=weights,
                        width=width, support_limit=support_limit, metadata=json.dumps(metadata))
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        print("Exported prediction-only model:", export_model(args.input, args.output))
    except (ValueError, KeyError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
