#!/usr/bin/env python3
"""Draw part 01 as seen from the robot side at the proposed mounting pose."""

from pathlib import Path
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle, FancyArrowPatch


HERE = Path(__file__).resolve().parent
OUT = HERE / "figures"


def arrow(ax, start, end, color="#24486c", width=2.4):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=18,
                                 linewidth=width, color=color))


def main():
    OUT.mkdir(exist_ok=True)
    fig, ax = plt.subplots(figsize=(11.5, 7.0), facecolor="white")
    fig.subplots_adjust(left=.055, right=.97, bottom=.16, top=.82)
    fig.text(.055, .945, "How to clock part 01 on the UR3e", fontsize=21, weight="bold", color="#20334a")
    fig.text(.055, .89, "View from the robot toward the printed frame  •  proposed bare-flange pose at (0, −0.4, 0.3) m",
             fontsize=11, color="#53657a")

    # CAD X–Z projection. These shapes and hole centers follow build_attachment.py.
    ax.add_patch(Polygon([(-90, -65), (-20, -70), (-20, -30), (-90, -35)],
                         closed=True, facecolor="#8ea3ba", edgecolor="#46617e", lw=2, zorder=1))
    ax.add_patch(Rectangle((-20, -70), 40, 40, facecolor="#20a99b", edgecolor="#136c64", lw=2, zorder=2))
    ax.add_patch(Circle((-90, -50), 31.5, facecolor="#627a98", edgecolor="#344d6a", lw=2.4, zorder=3))
    ax.add_patch(Circle((-90, -50), 10, facecolor="white", edgecolor="#344d6a", lw=1.3, zorder=4))
    for deg in (45, 135, 225, 315):
        x = -90 + 25*math.cos(math.radians(deg))
        z = -50 + 25*math.sin(math.radians(deg))
        ax.add_patch(Circle((x, z), 3.3, facecolor="white", edgecolor="#263f5c", lw=1.5, zorder=5))
    # Two overlapping bores and the short relief in the source CAD form this
    # optional pin-clearance feature near the CAD +Z edge of the plate.
    ax.add_patch(Rectangle((-93.3, -26), 6.6, 2, facecolor="white", edgecolor="#263f5c", lw=1, zorder=5))
    for zz in (-26, -24):
        ax.add_patch(Circle((-90, zz), 3.3, facecolor="white", edgecolor="#263f5c", lw=1, zorder=5))

    ax.plot([0, 0], [-106, 95], linestyle=(0, (5, 4)), color="#cf8240", lw=2, zorder=0)
    ax.add_patch(Circle((0, -100), 5, facecolor="#e6a24a", edgecolor="#a65f24", lw=1.5, zorder=4))
    ax.text(12, -100, "contact tip ↓", fontsize=12, weight="bold", va="center", color="#a65f24")
    arrow(ax, (0, -10), (0, 73), color="#cf8240")
    ax.text(12, 52, "cage + weights UP", fontsize=12, weight="bold", color="#a65f24")

    arrow(ax, (-90, -125), (20, -125))
    ax.text(-35, -142, "CAD +X  =  tool0 +X  =  Base +Y", fontsize=11, ha="center", color="#24486c")
    arrow(ax, (54, -66), (54, 25))
    ax.text(62, -20, "CAD +Z\n= Base +Z", fontsize=11, va="center", color="#24486c")
    ax.annotate("four M6 through holes\non 50 mm bolt circle", xy=(-107.7, -32.3), xytext=(-182, 20),
                fontsize=11, color="#263f5c", arrowprops={"arrowstyle": "->", "color": "#263f5c", "lw": 1.5})
    ax.annotate("upper pin clearance\n(if used)", xy=(-90, -25), xytext=(-181, 73),
                fontsize=11, color="#263f5c", arrowprops={"arrowstyle": "->", "color": "#263f5c", "lw": 1.5})
    ax.annotate("flat plate face\nagainst UR flange", xy=(-112, -62), xytext=(-184, -92),
                fontsize=11, color="#263f5c", arrowprops={"arrowstyle": "->", "color": "#263f5c", "lw": 1.5})
    ax.text(-25, -80, "bridge", fontsize=11, color="#344d6a")
    ax.text(22, -48, "guide axis", fontsize=11, color="#136c64")

    ax.set_xlim(-205, 132)
    ax.set_ylim(-153, 100)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.text(.055, .105,
             "Place the round plate against the robot flange. Rotate part 01 until the bridge points toward Base +Y and the pin slot is uppermost.",
             fontsize=10.6, color="#263f5c")
    fig.text(.055, .068,
             "Check that the four bolts align without force and that the guide axis is vertical. The physical flange may require a different wrist clocking.",
             fontsize=10.3, color="#53657a")
    for ext in ("png", "pdf"):
        path = OUT / f"r3_part01_mounting_orientation.{ext}"
        fig.savefig(path, dpi=220, facecolor="white")
        print(path)
    plt.close(fig)


if __name__ == "__main__":
    main()
