#!/usr/bin/env python3
"""Render the R3 printed parts at their CAD assembly positions.

Run with a Python environment containing numpy and matplotlib. The six STL
meshes are supplied in print orientation; this script reverses the rotations
and translations in R3/build_attachment.py before drawing them.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import colors
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


HERE = Path(__file__).resolve().parent
R3 = HERE.parent / "ur3e_weight_position_data_gather_attachment" / "R3"
OUT = HERE / "figures"
PARTS = [
    ("01_fixed_frame", "01  Frame / UR flange", "#627a98"),
    ("02_tray_plunger", "02  Tray + contact plunger", "#e6a24a"),
    ("03_guide_c030", "03  Guide", "#20a99b"),
    ("04_weight_cage", "04  Weight cage", "#dab74f"),
    ("05_top_cap", "05  Top cap", "#466a8a"),
    ("06_center_rod_3p6mm", "06  Center rod", "#b96cbb"),
]


def rotation(name: str) -> np.ndarray:
    if name == "01_fixed_frame":
        return np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)
    if name in ("02_tray_plunger", "03_guide_c030"):
        return np.diag([1.0, -1.0, -1.0])
    if name == "06_center_rod_3p6mm":
        return np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)
    return np.eye(3)


def read_stl(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    count = int.from_bytes(raw[80:84], "little")
    if len(raw) != 84 + count * 50:
        raise ValueError(f"Expected binary STL: {path}")
    face_dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(raw, dtype=face_dtype, count=count, offset=84)["vertices"].astype(float)


def restore_assembly_mesh(name: str, bbox: list[float]) -> np.ndarray:
    faces = read_stl(R3 / "stl" / f"{name}.stl")
    lo = np.array([bbox[0], bbox[2], bbox[4]])
    hi = np.array([bbox[1], bbox[3], bbox[5]])
    corners = np.array(np.meshgrid(*zip(lo, hi))).T.reshape(-1, 3)
    rot = rotation(name)
    rotated_box = corners @ rot.T
    shift = np.array([
        -(rotated_box[:, 0].min() + rotated_box[:, 0].max()) / 2,
        -(rotated_box[:, 1].min() + rotated_box[:, 1].max()) / 2,
        -rotated_box[:, 2].min(),
    ])
    assembled = (faces - shift) @ rot
    low = assembled.reshape(-1, 3).min(axis=0)
    high = assembled.reshape(-1, 3).max(axis=0)
    if not np.allclose(low, lo, atol=0.15) or not np.allclose(high, hi, atol=0.15):
        raise ValueError(f"Restored {name} bbox does not match CAD: {low}, {high}")
    return assembled


def cylinder_y(x: float, z: float, y0: float, y1: float, radius: float) -> np.ndarray:
    theta = np.linspace(0, 2 * np.pi, 25)
    rings = [np.column_stack((x + radius * np.cos(theta), np.full_like(theta, y), z + radius * np.sin(theta)))
             for y in (y0, y1)]
    return np.array([[rings[0][i], rings[0][i+1], rings[1][i+1]] for i in range(24)]
                    + [[rings[0][i], rings[1][i+1], rings[1][i]] for i in range(24)])


def mesh_artist(ax, faces: np.ndarray, color: str, alpha: float = 1.0, max_faces: int = 30000):
    if len(faces) > max_faces:
        faces = faces[np.linspace(0, len(faces) - 1, max_faces).astype(int)]
    ax.add_collection3d(Poly3DCollection(faces, facecolor=color, edgecolor="none", alpha=alpha,
                                         linewidths=0, zsort="average"))


def setup_3d(ax, *, xlim, ylim, zlim, elev=18, azim=-65):
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(*zlim)
    ax.set_box_aspect((xlim[1]-xlim[0], ylim[1]-ylim[0], zlim[1]-zlim[0]))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


def main() -> None:
    report = json.loads((R3 / "geometry_checks.json").read_text())
    meshes = {name: restore_assembly_mesh(name, report["parts"][name]["assembly_bbox_mm"])
              for name, _, _ in PARTS}
    OUT.mkdir(exist_ok=True)

    fig = plt.figure(figsize=(16, 10), facecolor="white")
    fig.text(.045, .955, "UR3e  |  R3 EIT weight attachment", fontsize=23, weight="bold", color="#20334a")
    fig.text(.045, .917, "Assembly positions from the R3 CAD • dimensions in mm • robot wrist shown schematically",
             fontsize=11, color="#64748b")

    ax1 = fig.add_axes([.02, .12, .46, .77], projection="3d")
    mesh_artist(ax1, cylinder_y(-90, -50, -78, 0, 31.5), "#b9c6d4")
    for name, _, color in PARTS:
        mesh_artist(ax1, meshes[name], color)
    setup_3d(ax1, xlim=(-145, 48), ylim=(-92, 55), zlim=(-125, 230), elev=16, azim=-62)
    fig.text(.05, .846, "A  Mounted at the UR3e tool flange", fontsize=15, weight="bold", color="#20334a")
    fig.text(.05, .815, "Four M6 bolts through the frame plate into the robot flange",
             fontsize=10.5, color="#506176")
    fig.text(.05, .18, "UR wrist  →  frame  →  vertical guide + floating weight head",
             fontsize=11, color="#253c55")
    fig.text(.05, .15, "At the revised mount pose: cage up; probe tip down; bridge toward Base +Y.",
             fontsize=10, color="#506176")

    ax2 = fig.add_axes([.46, .21, .31, .65], projection="3d")
    # Explode parts along their assembly axis. The rod is shifted sideways so its
    # position and full length remain legible next to the cage.
    offsets = {
        "01_fixed_frame": (0, 0, 0),
        "03_guide_c030": (0, 0, 45),
        "02_tray_plunger": (0, 0, 135),
        "04_weight_cage": (0, 0, 205),
        "06_center_rod_3p6mm": (72, 0, 205),
        "05_top_cap": (0, 0, 257),
    }
    for name, _, color in PARTS:
        mesh_artist(ax2, meshes[name] + np.array(offsets[name]), color)
    setup_3d(ax2, xlim=(-135, 100), ylim=(-52, 55), zlim=(-120, 480), elev=13, azim=-65)
    fig.text(.48, .846, "B  Parts before assembly", fontsize=15, weight="bold", color="#20334a")
    fig.text(.48, .815, "Lower parts onto the fixed frame along the guide axis", fontsize=10.5, color="#506176")

    key_x = .775
    fig.text(key_x, .85, "Printed parts", fontsize=15, weight="bold", color="#20334a")
    for i, (_, label, color) in enumerate(PARTS):
        y = .802 - i * .073
        fig.text(key_x, y, "■", color=color, fontsize=17, va="center")
        fig.text(key_x + .025, y, label, fontsize=10.5, va="center", color="#20334a")
    fig.text(key_x, .314, "Assembly order", fontsize=13, weight="bold", color="#20334a")
    steps = ["03 guide into 01 frame", "02 plunger through guide", "04 cage onto tray", "06 rod into tray; add weights", "05 cap onto cage and rod", "01 frame onto UR tool flange"]
    for i, line in enumerate(steps):
        fig.text(key_x, .281-i*.031, f"{i+1}. {line}", fontsize=9.6, color="#334b63")
    fig.text(.045, .056,
             "Orientation: UR tool0 +Z = Base −X; CAD +Y follows tool0 +Z; CAD +X points toward Base +Y; CAD +Z points up.",
             fontsize=10.3, color="#394e64")
    fig.text(.045, .029, "Fit and fastener details: R3 README. Verify the physical flange clocking and clearance before moving the loaded robot.",
             fontsize=9.8, color="#64748b")
    for ext in ("png", "pdf"):
        path = OUT / f"r3_ur3e_attachment_assembly.{ext}"
        fig.savefig(path, dpi=220, facecolor="white")
        print(path)
    plt.close(fig)


if __name__ == "__main__":
    main()
