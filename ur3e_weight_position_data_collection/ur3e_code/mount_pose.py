"""Move the bare UR3e flange to a CAD-oriented R3 mounting pose.

The proposed Base pose is near (0, -0.4, 0.3) m. A custom pendant-taught
pose remains optional.
"""
from __future__ import annotations

import argparse
import json
import secrets
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from collect import (HERE, RTDE, dashboard_info, fresh_state, save_json,
                     stop_running_program, upload_script, vector)
from ur3e_demo.common import pose_matrix

# Tool0 columns in UR Base after a 180-degree turn about Base Z:
# X=+Base Y, Y=-Base Z, Z=-Base X.
# Mount R3 with CAD +Y along tool0 +Z and CAD +X along tool0 +X.
MOUNT_TCP_BASE_POSE = np.array([0.0, -0.4, 0.3,
                                -1.209199576156145, -1.209199576156145,
                                1.209199576156145])
MOUNT_CAD_TIP_BASE_M = np.array([0.0, -0.31, 0.25])


def pose_errors(actual, target=MOUNT_TCP_BASE_POSE):
    actual = vector(actual, 6, "actual TCP pose")
    target = vector(target, 6, "target TCP pose")
    ra, rt = pose_matrix(actual)[:3, :3], pose_matrix(target)[:3, :3]
    angle = np.arccos(np.clip((np.trace(rt.T @ ra) - 1) / 2, -1, 1))
    return float(np.linalg.norm(actual[:3] - target[:3])), float(angle)


def bare_joint_state(rtde):
    states = []
    end = time.monotonic() + 0.5
    while time.monotonic() < end or len(states) < 4:
        state = fresh_state(rtde)
        tcp_offset = vector(state["tcp_offset"], 6, "active TCP offset")
        if np.linalg.norm(tcp_offset) > 1e-5:
            raise RuntimeError(
                f"Active TCP offset is {tcp_offset.tolist()} [m, rad]. "
                "With the attachment removed, select a zero-offset TCP on the pendant "
                "(X, Y, Z, RX, RY, RZ = 0), then retry mounting-pose work"
            )
        if np.max(np.abs(vector(state["actual_qd"], 6, "joint speed"))) > 0.01:
            raise RuntimeError("Robot must be stationary")
        states.append(state)
    joints = np.median([s["actual_q"] for s in states], axis=0)
    if np.max(np.linalg.norm(np.asarray([s["actual_q"] for s in states]) - joints, axis=1)) > 0.002:
        raise RuntimeError("Joint pose changed during recording")
    return states[-1], joints


def set_bare_flange_tcp(rtde, robot_ip):
    """Set the robot's active TCP to tool0, then confirm it through RTDE."""
    state = fresh_state(rtde)
    offset = vector(state["tcp_offset"], 6, "active TCP offset")
    if np.linalg.norm(offset) <= 1e-5:
        return
    if np.max(np.abs(vector(state["actual_qd"], 6, "joint speed"))) > 0.01:
        raise RuntimeError("Robot must be stationary before changing the active TCP")
    if stop_running_program(robot_ip):
        print("Stopped existing robot program before setting the bare-flange TCP.")
    token = secrets.randbelow(1_000_000) + 1000
    script = ("def eit_set_bare_flange_tcp():\n"
              f"  write_output_integer_register(25, {token})\n"
              "  write_output_integer_register(24, 0)\n"
              "  set_tcp(p[0, 0, 0, 0, 0, 0])\n"
              f"  write_output_integer_register(24, {token})\n"
              "end\n")
    conn = upload_script(robot_ip, script)
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            conn.check()
            sample = fresh_state(rtde, timeout=1)
            if np.max(np.abs(vector(sample["actual_qd"], 6, "joint speed"))) > 0.01:
                raise RuntimeError("Robot moved while setting the active TCP")
            if (int(sample["output_int_register_25"]) == token and
                    int(sample["output_int_register_24"]) == token and
                    np.linalg.norm(vector(sample["tcp_offset"], 6, "active TCP offset")) <= 1e-5):
                print(f"Set active TCP from {offset.tolist()} to the bare flange (zero offset).")
                return
        raise TimeoutError("Robot did not confirm a zero active TCP offset")
    finally:
        conn.close()


def record(args):
    path = Path(args.pose_file)
    if path.exists() and not args.replace:
        raise FileExistsError(f"{path} exists; choose another file or pass --replace")
    info = dashboard_info(args.robot_ip)
    if info["robot_model"].lower() not in ("ur3", "ur3e"):
        raise RuntimeError(f"Expected UR3e; got {info['robot_model']}")
    print("With the R3 attachment removed, jog to an accessible mounting pose, stop, then press Enter.")
    input()
    rtde = RTDE(args.robot_ip, hz=125, writable=False)
    try:
        state, joints = bare_joint_state(rtde)
        data = {"robot_ip": args.robot_ip, "joint_pose_rad": joints.tolist(),
                "bare_flange_tcp_base_pose": vector(state["actual_TCP_pose"], 6, "TCP pose").tolist(),
                "active_tcp_offset_pose": vector(state["tcp_offset"], 6, "TCP offset").tolist(),
                "payload_kg_at_teach": float(state["payload"]),
                "recorded_utc": datetime.now(timezone.utc).isoformat()}
        save_json(path, data)
        print(f"Saved mounting pose in {path}: joint angles {data['joint_pose_rad']}")
    finally:
        rtde.close()


def move_script(joints, token, speed, acceleration):
    vals = ", ".join(f"{float(q):.9f}" for q in joints)
    return ("def eit_bare_flange_mount_pose():\n"
            f"  write_output_integer_register(25, {token})\n"
            "  write_output_integer_register(24, 0)\n"
            f"  movej([{vals}], a={acceleration:.6f}, v={speed:.6f})\n"
            f"  write_output_integer_register(24, {token})\n"
            "end\n")


def proposed_move_script(token):
    vals = ", ".join(f"{float(v):.9f}" for v in MOUNT_TCP_BASE_POSE)
    return ("def eit_proposed_bare_flange_pose():\n"
            f"  write_output_integer_register(25, {token})\n"
            "  write_output_integer_register(24, 0)\n"
            "  set_tcp(p[0, 0, 0, 0, 0, 0])\n"
            f"  target = p[{vals}]\n"
            "  q_start = get_actual_joint_positions()\n"
            "  if get_inverse_kin_has_solution(target, q_start):\n"
            "    q_goal = get_inverse_kin(target, q_start)\n"
            "    movej(q_goal, a=0.200000, v=0.100000)\n"
            f"    write_output_integer_register(24, {token})\n"
            "  else:\n"
            "    write_output_integer_register(24, -1)\n"
            "  end\n"
            "end\n")


def move_to_proposed(args):
    info = dashboard_info(args.robot_ip)
    if info["robot_model"].lower() not in ("ur3", "ur3e") or info["remote_control"].lower() != "true":
        raise RuntimeError(f"Expected UR3e in Remote Control; got {info}")
    rtde = RTDE(args.robot_ip, hz=125, writable=False)
    conn = None
    try:
        set_bare_flange_tcp(rtde, args.robot_ip)
        state, _ = bare_joint_state(rtde)
        position_error, rotation_error = pose_errors(state["actual_TCP_pose"])
        print("Proposed bare-flange TCP in UR Base [m, rad]:", MOUNT_TCP_BASE_POSE.tolist())
        print("R3 CAD tip if mounted at the specified clocking [m]:", MOUNT_CAD_TIP_BASE_M.tolist())
        print(f"Current difference: {position_error:.3f} m, {np.degrees(rotation_error):.1f} degrees")
        if not args.execute:
            print("Inspection only. Add --execute --path-clear-verified after checking the full joint-space path.")
            return
        if not args.path_clear_verified:
            raise ValueError("Inspect the full bare-flange path and pass --path-clear-verified")
        if stop_running_program(args.robot_ip):
            print("Stopped existing robot program.")
        bare_joint_state(rtde)
        token = secrets.randbelow(1_000_000) + 1000
        conn = upload_script(args.robot_ip, proposed_move_script(token))
        deadline = time.monotonic() + 180
        started = False
        while time.monotonic() < deadline:
            conn.check()
            sample = fresh_state(rtde, timeout=1)
            if np.max(np.abs(sample["actual_qd"])) > 0.35:
                raise RuntimeError("Measured joint speed exceeded 0.35 rad/s")
            started |= int(sample["output_int_register_25"]) == token
            status = int(sample["output_int_register_24"])
            if started and status == -1:
                raise RuntimeError("Robot controller found no inverse-kinematics solution")
            if started and status == token:
                arrived_state, _ = bare_joint_state(rtde)
                pos, angle = pose_errors(arrived_state["actual_TCP_pose"])
                if pos > 0.003 or angle > 0.03:
                    raise RuntimeError(f"Mounting pose error too large: {pos:.4f} m, {angle:.4f} rad")
                print("Robot reached the proposed bare-flange mounting pose.")
                return
        raise TimeoutError("Proposed mounting-pose move timed out")
    except BaseException:
        if conn is not None:
            try:
                stop_running_program(args.robot_ip)
            except OSError:
                pass
        raise
    finally:
        if conn is not None:
            conn.close()
        rtde.close()


def return_to_mount(args):
    saved = json.loads(Path(args.pose_file).read_text())
    if saved["robot_ip"] != args.robot_ip:
        raise ValueError("Saved mounting pose belongs to another robot IP")
    target = vector(saved["joint_pose_rad"], 6, "saved joint pose")
    if np.linalg.norm(vector(saved["active_tcp_offset_pose"], 6, "saved TCP offset")) > 1e-5:
        raise ValueError("Saved mounting pose was not recorded with the bare flange TCP")
    info = dashboard_info(args.robot_ip)
    if info["robot_model"].lower() not in ("ur3", "ur3e") or info["remote_control"].lower() != "true":
        raise RuntimeError(f"Expected UR3e in Remote Control; got {info}")
    rtde = RTDE(args.robot_ip, hz=125, writable=False)
    conn = None
    try:
        state, start = bare_joint_state(rtde)
        delta = np.abs(start - target)
        print(f"Current joints (rad): {np.round(start, 4).tolist()}")
        print(f"Saved joints   (rad): {np.round(target, 4).tolist()}")
        print(f"Largest joint change: {np.max(delta):.4f} rad")
        if np.max(delta) > 0.2:
            raise RuntimeError("More than 0.2 rad from the taught pose; move near it with the pendant first")
        if not args.execute:
            print("Pose check complete. Add --execute to move from this nearby pose.")
            return
        if not args.path_clear_verified:
            raise ValueError("Inspect the bare-flange path and pass --path-clear-verified")
        if abs(float(state["payload"]) - float(saved["payload_kg_at_teach"])) > 0.02:
            raise RuntimeError("Payload differs from the saved bare-flange mounting setup")
        if stop_running_program(args.robot_ip):
            print("Stopped existing robot program.")
        bare_joint_state(rtde)
        token = secrets.randbelow(1_000_000) + 1000
        conn = upload_script(args.robot_ip, move_script(target, token, .1, .2))
        deadline = time.monotonic() + 20
        started = False
        while time.monotonic() < deadline:
            conn.check()
            sample = fresh_state(rtde, timeout=1)
            if np.max(np.abs(sample["actual_qd"])) > 0.35:
                raise RuntimeError("Measured joint speed exceeded 0.35 rad/s")
            started |= int(sample["output_int_register_25"]) == token
            if started and int(sample["output_int_register_24"]) == token:
                _, arrived = bare_joint_state(rtde)
                if np.max(np.abs(arrived - target)) > 0.01:
                    raise RuntimeError("Robot stopped outside the saved mounting pose")
                print("Robot reached the saved bare-flange mounting pose.")
                return
        raise TimeoutError("Mounting-pose move timed out")
    except BaseException:
        if conn is not None:
            try:
                stop_running_program(args.robot_ip)
            except OSError:
                pass
        raise
    finally:
        if conn is not None:
            conn.close()
        rtde.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default=None,
                        help="Override robot_ip from this folder's config.json")
    parser.add_argument("--pose-file", default=str(HERE / "mount_pose.json"))
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("record", help="Optionally save a custom bare-flange pose")
    r.add_argument("--replace", action="store_true")
    m = sub.add_parser("move", help="Inspect or move to the proposed R3 mounting pose")
    m.add_argument("--execute", action="store_true")
    m.add_argument("--path-clear-verified", action="store_true",
                   help="Operator checked the bare-flange route to the proposed pose")
    s = sub.add_parser("move-saved", help="Return to an optional pendant-taught pose")
    s.add_argument("--execute", action="store_true")
    s.add_argument("--path-clear-verified", action="store_true")
    args = parser.parse_args()
    if args.robot_ip is None:
        args.robot_ip = json.loads((HERE / "config.json").read_text()).get("robot_ip")
    if not args.robot_ip:
        parser.error("Set robot_ip in config.json or pass --robot-ip")
    if args.command == "move-saved" and not Path(args.pose_file).is_file():
        parser.error(f"No taught mounting pose at {args.pose_file}. With the attachment off, "
                     "jog the bare flange to an accessible pose and run 'python mount_pose.py record' first.")
    if args.command == "record":
        record(args)
    elif args.command == "move-saved":
        return_to_mount(args)
    else:
        move_to_proposed(args)


if __name__ == "__main__":
    main()
