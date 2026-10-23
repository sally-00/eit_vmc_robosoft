"""UR3e deadweight EIT position scan with Amodo USB acquisition.

Robot positions use UR Base metres. A motion-only run requires the explicit
--metadata-only flag and stores null electrical measurements.
"""
from __future__ import annotations

import argparse
import json
import math
import secrets
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DEMO = HERE.parents[2] / "00_VMC_demo_ur3e"
sys.path.insert(0, str(DEMO))
from ur3e_demo.rtde import RTDE, dashboard_info, stop_running_program, upload_script  # noqa: E402
from eit_reader import EITReader  # noqa: E402

MARKS = ("east", "west", "north", "south", "contact")


def vector(value, n, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (n,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {n} finite numbers")
    return result


def load_config(path):
    cfg = json.loads(Path(path).read_text())
    required = ("robot_ip", "active_radius_m", "edge_margin_m", "grid_spacing_m",
                "lift_m", "speed_m_s", "acceleration_m_s2", "settle_s",
                "pose_tolerance_m", "orientation_tolerance_rad", "max_move_s",
                "expected_tcp_offset_pose", "probe_axis_in_tcp", "expected_payload_kg",
                "expected_payload_cog_m")
    for key in required:
        if key not in cfg or cfg[key] is None:
            raise ValueError(f"Set {key} in {path}")
    vector(cfg["expected_tcp_offset_pose"], 6, "expected_tcp_offset_pose")
    axis = vector(cfg["probe_axis_in_tcp"], 3, "probe_axis_in_tcp")
    vector(cfg["expected_payload_cog_m"], 3, "expected_payload_cog_m")
    if abs(np.linalg.norm(axis) - 1) > 0.01:
        raise ValueError("probe_axis_in_tcp must be a unit vector measured for the mounted R3 fixture")
    offset = vector(cfg["expected_tcp_offset_pose"], 6, "expected_tcp_offset_pose")
    if not 0.08 <= np.linalg.norm(offset[:2]) <= 0.11:
        raise ValueError("R3 probe axis is 90 mm from flange center; verify the active TCP offset")
    if not isinstance(cfg["robot_ip"], str) or not cfg["robot_ip"]:
        raise ValueError("Set robot_ip")
    for key in required[1:-4]:
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f"{key} must be positive and finite")
    if cfg["edge_margin_m"] >= cfg["active_radius_m"] or cfg["grid_spacing_m"] > 2 * cfg["active_radius_m"]:
        raise ValueError("Invalid scan radius or grid spacing")
    if cfg["speed_m_s"] > 0.02 or cfg["acceleration_m_s2"] > 0.1:
        raise ValueError("This collector limits speed to 20 mm/s and acceleration to 100 mm/s^2")
    if cfg["lift_m"] < 0.005:
        raise ValueError("lift_m must be at least 5 mm")
    if not math.isfinite(cfg["expected_payload_kg"]) or cfg["expected_payload_kg"] <= 0:
        raise ValueError("Set expected_payload_kg for the installed fixture and weight")
    return cfg


def fresh_state(rtde, timeout=2):
    state = rtde.state(timeout=timeout)
    vector(state["actual_TCP_pose"], 6, "actual_TCP_pose")
    if int(state["robot_mode"]) != 7 or not int(state["safety_status_bits"]) & 1 or int(state["safety_status_bits"]) & 0x7fc:
        raise RuntimeError(f"Robot not in normal running state: mode={state['robot_mode']}, safety={state['safety_status_bits']}")
    return state


def rotated_axis(rotvec, axis):
    rotvec = vector(rotvec, 3, "TCP rotation vector")
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return axis
    k = rotvec / angle
    return axis * math.cos(angle) + np.cross(k, axis) * math.sin(angle) + k * (k @ axis) * (1 - math.cos(angle))


def check_fixture_orientation(pose, cfg):
    axis = rotated_axis(pose[3:], vector(cfg["probe_axis_in_tcp"], 3, "probe_axis_in_tcp"))
    tilt = math.degrees(math.acos(float(np.clip(axis @ np.array([0., 0., -1.]), -1, 1))))
    if tilt > 2.0:
        raise RuntimeError(f"R3 probe axis is tilted {tilt:.2f} degrees; limit is 2 degrees")
    # The R3 flange face is perpendicular to CAD Y; its normal is perpendicular
    # to the probe axis. The UR tool0 Z axis is the flange normal.
    flange_in_tcp = rotated_axis(-vector(cfg["expected_tcp_offset_pose"][3:], 3, "TCP offset rotation"),
                                 np.array([0., 0., 1.]))
    flange_in_base = rotated_axis(pose[3:], flange_in_tcp)
    if abs(float(flange_in_base @ axis)) > math.sin(math.radians(3)):
        raise RuntimeError("Flange normal is not perpendicular to the R3 probe axis; check mounting and TCP rotation")
    return tilt


def stationary_pose(rtde, cfg, duration=0.5):
    poses = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline or len(poses) < 4:
        s = fresh_state(rtde)
        if not np.allclose(s["tcp_offset"], cfg["expected_tcp_offset_pose"], atol=1e-5, rtol=0):
            raise RuntimeError("Active TCP differs from expected_tcp_offset_pose")
        if abs(s["payload"] - cfg["expected_payload_kg"]) > 0.02 or not np.allclose(
                s["payload_cog"], cfg["expected_payload_cog_m"], atol=0.005, rtol=0):
            raise RuntimeError("Active payload or center of gravity differs from config")
        if np.linalg.norm(s["actual_TCP_speed"][:3]) > 0.002 or np.linalg.norm(s["actual_qd"]) > 0.03:
            raise RuntimeError("Robot is moving; wait for it to stop")
        poses.append(s["actual_TCP_pose"])
    median = np.median(poses, axis=0)
    if np.max(np.linalg.norm(np.asarray(poses)[:, :3] - median[:3], axis=1)) > 0.001:
        raise RuntimeError("TCP moved during capture")
    check_fixture_orientation(median, cfg)
    return median.tolist()


def save_json(path, obj):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n")
    temp.replace(path)


def calibration_from_marks(marks, cfg):
    pts = {name: vector(marks[name], 6, name) for name in MARKS}
    for pose in pts.values():
        check_fixture_orientation(pose, cfg)
    east, west, north, south = (pts[n][:3] for n in MARKS[:4])
    ex = east - west
    ny = north - south
    ex[2] = ny[2] = 0
    if np.linalg.norm(ex) < 0.04 or np.linalg.norm(ny) < 0.04:
        raise ValueError("Rim marks are too close together")
    x = ex / np.linalg.norm(ex)
    y0 = ny / np.linalg.norm(ny)
    if abs(float(x @ y0)) > 0.1:
        raise ValueError("Rim axes are not perpendicular within 0.1 dot product")
    y = y0 - x * float(x @ y0)
    y /= np.linalg.norm(y)
    c1, c2 = (east + west) / 2, (north + south) / 2
    if np.linalg.norm((c1 - c2)[:2]) > 0.003:
        raise ValueError("Opposite rim midpoints differ by more than 3 mm")
    center = (c1 + c2) / 2
    radial = [np.linalg.norm((pts[n][:3] - center)[:2]) for n in MARKS[:4]]
    if max(abs(r - cfg["active_radius_m"]) for r in radial) > 0.005:
        raise ValueError("Marked rim differs from configured active radius by more than 5 mm")
    if max(abs(pts[n][2] - center[2]) for n in MARKS[:4]) > 0.003:
        raise ValueError("Rim captures have inconsistent height")
    contact = pts["contact"]
    if np.linalg.norm((contact[:3] - center)[:2]) > 0.003:
        raise ValueError("Contact reference is more than 3 mm from calculated center")
    if max(np.linalg.norm(pts[n][3:] - contact[3:]) for n in MARKS[:4]) > 0.05:
        raise ValueError("TCP orientation changed during center calibration")
    if not 0.003 <= center[2] - contact[2] <= 0.1:
        raise ValueError("Contact reference must be 3–100 mm below rim hover height")
    return {"frame": "UR Base", "units": "m", "center_hover_base_m": center.tolist(),
            "center_contact_tcp_pose": contact.tolist(), "x_axis_base": x.tolist(),
            "y_axis_base": y.tolist(), "marked_radius_m": radial,
            "expected_tcp_offset_pose": cfg["expected_tcp_offset_pose"],
            "probe_axis_in_tcp": cfg["probe_axis_in_tcp"],
            "robot_ip": cfg["robot_ip"], "marks": marks,
            "calibrated_utc": datetime.now(timezone.utc).isoformat()}


def calibrate(args, cfg):
    path = Path(args.calibration)
    data = json.loads(path.read_text()) if path.exists() else {"marks": {}}
    if "robot_ip" in data and data["robot_ip"] != cfg["robot_ip"]:
        raise ValueError("Existing calibration belongs to a different robot")
    if "expected_tcp_offset_pose" in data and data["expected_tcp_offset_pose"] != cfg["expected_tcp_offset_pose"]:
        raise ValueError("Existing calibration uses a different TCP")
    if "probe_axis_in_tcp" in data and data["probe_axis_in_tcp"] != cfg["probe_axis_in_tcp"]:
        raise ValueError("Existing calibration uses a different probe orientation")
    print(f"Jog the mounted attachment over the '{args.mark}' mark with the pendant, stop motion, then press Enter to record; Ctrl+C cancels.")
    input()
    with_rtde = RTDE(cfg["robot_ip"], hz=125, writable=False)
    try:
        pose = stationary_pose(with_rtde, cfg)
    finally:
        with_rtde.close()
    data.update({"robot_ip": cfg["robot_ip"], "expected_tcp_offset_pose": cfg["expected_tcp_offset_pose"],
                 "probe_axis_in_tcp": cfg["probe_axis_in_tcp"]})
    data["marks"][args.mark] = pose
    if all(m in data["marks"] for m in MARKS):
        data = calibration_from_marks(data["marks"], cfg)
    save_json(path, data)
    print(f"Saved {args.mark} TCP pose in {path}: {pose}")
    if "center_hover_base_m" in data:
        print(f"Calculated pad center (Base, m): {data['center_hover_base_m'][:3]}")


def inspect_pose(cfg):
    rtde = RTDE(cfg["robot_ip"], hz=125, writable=False)
    try:
        pose = stationary_pose(rtde, cfg)
        print(json.dumps({"tcp_base_pose_m_rad": pose,
                          "probe_down_tilt_deg": check_fixture_orientation(pose, cfg),
                          "expected_tcp_offset_pose": cfg["expected_tcp_offset_pose"],
                          "expected_payload_kg": cfg["expected_payload_kg"],
                          "expected_payload_cog_m": cfg["expected_payload_cog_m"]}, indent=2))
    finally:
        rtde.close()


def grid(cfg):
    radius = cfg["active_radius_m"] - cfg["edge_margin_m"] - 0.005  # 10 mm flat contact face
    step = cfg["grid_spacing_m"]
    extent = math.floor(radius / step)
    points = [(i * step, j * step) for j in range(-extent, extent + 1)
              for i in range(-extent, extent + 1) if math.hypot(i * step, j * step) <= radius + 1e-9]
    return sorted(points, key=lambda p: (p[1], p[0]))


def target_pose(cal, xy, z):
    base = np.asarray(cal["center_contact_tcp_pose"], float)
    base[:3] += xy[0] * vector(cal["x_axis_base"], 3, "x_axis_base") + xy[1] * vector(cal["y_axis_base"], 3, "y_axis_base")
    base[2] += z
    return base


def motion_script(pose, cfg, token):
    vals = ", ".join(f"{float(v):.9f}" for v in pose)
    return ("def eit_position_step():\n"
            f"  write_output_integer_register(25, {token})\n"
            "  write_output_integer_register(24, 0)\n"
            f"  movel(p[{vals}], a={cfg['acceleration_m_s2']:.6f}, v={cfg['speed_m_s']:.6f})\n"
            f"  write_output_integer_register(24, {token})\n"
            "end\n")


def move(rtde, cfg, pose, token):
    start = fresh_state(rtde)
    check_fixture_orientation(pose, cfg)
    if np.linalg.norm(start["actual_TCP_pose"][:3] - pose[:3]) > 0.06:
        raise RuntimeError("Move exceeds 60 mm; reposition with pendant before scan")
    if np.linalg.norm(start["actual_TCP_pose"][3:] - pose[3:]) > cfg["orientation_tolerance_rad"]:
        raise RuntimeError("TCP orientation differs from calibration")
    conn = upload_script(cfg["robot_ip"], motion_script(pose, cfg, token))
    deadline = time.monotonic() + cfg["max_move_s"]
    started = False
    try:
        while time.monotonic() < deadline:
            conn.check()
            state = fresh_state(rtde, timeout=1)
            started |= int(state["output_int_register_25"]) == token
            if started and int(state["output_int_register_24"]) == token:
                actual = stationary_pose(rtde, cfg)
                if np.linalg.norm(np.asarray(actual[:3]) - pose[:3]) > cfg["pose_tolerance_m"]:
                    raise RuntimeError("Position tolerance exceeded after movel")
                if np.linalg.norm(np.asarray(actual[3:]) - pose[3:]) > cfg["orientation_tolerance_rad"]:
                    raise RuntimeError("Orientation tolerance exceeded after movel")
                return actual
        raise TimeoutError("Motion completion timeout")
    except BaseException:
        stop_running_program(cfg["robot_ip"])
        raise
    finally:
        conn.close()


def append_record(path, record):
    with Path(path).open("a") as stream:
        stream.write(json.dumps(record, allow_nan=False) + "\n")
        stream.flush()


def scan(args, cfg):
    cal = json.loads(Path(args.calibration).read_text())
    if "center_hover_base_m" not in cal:
        raise ValueError("Calibration incomplete; capture all five marks")
    if (cal["robot_ip"] != cfg["robot_ip"] or
            cal["expected_tcp_offset_pose"] != cfg["expected_tcp_offset_pose"] or
            cal["probe_axis_in_tcp"] != cfg["probe_axis_in_tcp"]):
        raise ValueError("Calibration robot, TCP or probe axis does not match config")
    check_fixture_orientation(cal["center_contact_tcp_pose"], cfg)
    if (not args.weight_id.strip() or not math.isfinite(args.added_mass_kg) or
            not math.isfinite(args.moving_mass_kg) or
            not 0 <= args.added_mass_kg <= 1 or args.moving_mass_kg <= 0):
        raise ValueError("Provide weighed moving head mass and 0–1 kg added mass")
    points = grid(cfg)
    print(f"Scan: {len(points)} positions; nominal mass {args.moving_mass_kg + args.added_mass_kg:.4f} kg")
    if not args.execute:
        for i, xy in enumerate(points):
            print(f"{i:03d}: pad x={xy[0]*1000:+.1f} mm y={xy[1]*1000:+.1f} mm, contact TCP={target_pose(cal, xy, 0).round(5).tolist()}")
        return
    if not args.fixture_clearance_verified:
        raise ValueError("Inspect the R3 fixture sweep and pass --fixture-clearance-verified")
    reader = None if args.metadata_only else EITReader(cfg)
    info = dashboard_info(cfg["robot_ip"])
    if info["robot_model"].lower() not in ("ur3", "ur3e") or info["remote_control"].lower() != "true":
        raise RuntimeError(f"Expected UR3e in Remote Control; got {info}")
    run_dir = Path(args.output_root) / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + secrets.token_hex(4))
    run_dir.mkdir(parents=True, exist_ok=False)
    manifest = {"status": "started", "utc": datetime.now(timezone.utc).isoformat(),
                "weight_id": args.weight_id, "added_mass_kg": args.added_mass_kg,
                "moving_mass_kg": args.moving_mass_kg, "nominal_force_N": 9.81 * (args.added_mass_kg + args.moving_mass_kg),
                "calibration": cal, "config": cfg, "position_count": len(points),
                "eit_status": "absent" if args.metadata_only else "Amodo ARIA EITSYS",
                "eit_device": None}
    save_json(run_dir / "manifest.json", manifest)
    print(f"Run directory: {run_dir}")
    rtde = None
    try:
        rtde = RTDE(cfg["robot_ip"], hz=125, writable=False)
        if stop_running_program(cfg["robot_ip"]):
            print("Stopped existing robot program.")
        travel0 = target_pose(cal, (0, 0), cfg["lift_m"])
        pose0 = stationary_pose(rtde, cfg)
        if np.linalg.norm(np.asarray(pose0[:3]) - travel0[:3]) > cfg["pose_tolerance_m"]:
            raise RuntimeError("Start at calibrated center travel pose with pendant")
        if np.linalg.norm(np.asarray(pose0[3:]) - travel0[3:]) > cfg["orientation_tolerance_rad"]:
            raise RuntimeError("Start TCP orientation differs from calibration")
        if reader is not None:
            reader.open()  # Autogain at the verified raised no-contact start pose.
            manifest["eit_device"] = reader.metadata()
            save_json(run_dir / "manifest.json", manifest)
            rtde.close()
            rtde = RTDE(cfg["robot_ip"], hz=125, writable=False)
            stationary_pose(rtde, cfg)
        token = secrets.randbelow(1_000_000) + 1000
        for index, xy in enumerate(points):
            travel = target_pose(cal, xy, cfg["lift_m"])
            contact = target_pose(cal, xy, 0)
            move(rtde, cfg, travel, token + 3 * index)
            if reader is not None:
                time.sleep(cfg["settle_s"])
            baseline = None if reader is None else reader.capture()
            baseline_utc = datetime.now(timezone.utc).isoformat()
            actual = move(rtde, cfg, contact, token + 3 * index + 1)
            time.sleep(cfg["settle_s"])
            settled = stationary_pose(rtde, cfg)
            measurement = None if reader is None else reader.capture()
            contact_utc = datetime.now(timezone.utc).isoformat()
            record = {"index": index, "pad_xy_m": list(xy), "target_tcp_base_m": contact.tolist(),
                      "arrived_tcp_base_m": actual, "settled_tcp_base_m": settled,
                      "baseline_utc": baseline_utc, "contact_utc": contact_utc,
                      "eit_baseline": baseline, "eit_contact": measurement}
            append_record(run_dir / "samples.jsonl", record)
            move(rtde, cfg, travel, token + 3 * index + 2)
            print(f"Saved position {index + 1}/{len(points)}")
        manifest["status"] = "complete"
    except BaseException as error:
        manifest["status"] = "interrupted"
        manifest["error"] = str(error)
        try:
            stop_running_program(cfg["robot_ip"])
        except OSError:
            pass
        raise
    finally:
        save_json(run_dir / "manifest.json", manifest)
        if rtde is not None:
            rtde.close()
        if reader is not None:
            reader.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(HERE / "config.json"))
    parser.add_argument("--calibration", default=str(HERE / "pad_calibration.json"))
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("inspect-pose", help="Read-only check of R3 probe pose, TCP and payload")
    c = sub.add_parser("calibrate", help="Read a stationary TCP pose; pendant motion only")
    c.add_argument("mark", choices=MARKS)
    s = sub.add_parser("scan", help="Print scan plan or run robot positions")
    s.add_argument("--weight-id", required=True)
    s.add_argument("--added-mass-kg", type=float, required=True)
    s.add_argument("--moving-mass-kg", type=float, required=True)
    s.add_argument("--output-root", default=str(HERE / "runs"))
    s.add_argument("--execute", action="store_true")
    s.add_argument("--metadata-only", action="store_true", help="Robot motion check without EIT data")
    s.add_argument("--fixture-clearance-verified", action="store_true",
                   help="Operator checked the full R3 cage, frame and robot sweep")
    args = parser.parse_args()
    cfg = load_config(args.config)
    if args.command == "inspect-pose":
        inspect_pose(cfg)
    elif args.command == "calibrate":
        calibrate(args, cfg)
    else:
        scan(args, cfg)


if __name__ == "__main__":
    main()
