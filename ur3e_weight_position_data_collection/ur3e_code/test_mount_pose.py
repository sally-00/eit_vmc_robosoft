"""Offline checks for the taught bare-flange mounting move."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np

import mount_pose


class MountPoseTests(unittest.TestCase):
    def test_missing_taught_pose_explains_record_step_without_connecting(self):
        with tempfile.TemporaryDirectory() as folder:
            result = subprocess.run(
                [sys.executable, str(Path(mount_pose.__file__)), "--pose-file",
                 str(Path(folder) / "missing.json"), "move-saved"],
                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("python mount_pose.py record", result.stderr)
        self.assertNotIn("Traceback", result.stderr)

    def test_far_start_rejects_motion_before_upload(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "pose.json"
            path.write_text(json.dumps({"robot_ip": "192.0.2.1", "joint_pose_rad": [.25, 0, 0, 0, 0, 0],
                                        "active_tcp_offset_pose": [0] * 6, "payload_kg_at_teach": 0}))
            args = SimpleNamespace(robot_ip="192.0.2.1", pose_file=str(path), execute=True,
                                   path_clear_verified=True)
            robot = MagicMock()
            with patch.object(mount_pose, "dashboard_info", return_value={"robot_model": "UR3e", "remote_control": "true"}), \
                 patch.object(mount_pose, "RTDE", return_value=robot), \
                 patch.object(mount_pose, "bare_joint_state", return_value=({"payload": 0}, np.zeros(6))), \
                 patch.object(mount_pose, "upload_script") as upload:
                with self.assertRaisesRegex(RuntimeError, "More than 0.2 rad"):
                    mount_pose.return_to_mount(args)
                upload.assert_not_called()
                robot.close.assert_called_once()

    def test_move_script_uses_joint_position_control(self):
        script = mount_pose.move_script(np.zeros(6), 1234, .1, .2)
        self.assertIn("movej([", script)
        self.assertIn("a=0.200000, v=0.100000", script)
        self.assertIn("write_output_integer_register(24, 1234)", script)

    def test_nonzero_tcp_is_set_to_zero_and_confirmed_without_motion(self):
        initial = {"tcp_offset": [0, 0, .0203, 0, 0, 0], "actual_qd": [0] * 6}
        confirmed = {**initial, "tcp_offset": [0] * 6,
                     "output_int_register_25": 1234, "output_int_register_24": 1234}
        conn = MagicMock()
        with patch.object(mount_pose, "fresh_state", side_effect=[initial, confirmed]), \
             patch.object(mount_pose, "stop_running_program", return_value=False), \
             patch.object(mount_pose, "secrets") as secrets, \
             patch.object(mount_pose, "upload_script", return_value=conn) as upload:
            secrets.randbelow.return_value = 234
            mount_pose.set_bare_flange_tcp(MagicMock(), "192.0.2.1")
        script = upload.call_args.args[1]
        self.assertIn("set_tcp(p[0, 0, 0, 0, 0, 0])", script)
        self.assertNotIn("movej(", script)
        self.assertNotIn("movel(", script)
        conn.close.assert_called_once()

    def test_proposed_rotation_points_r3_probe_down_and_tip_toward_pad(self):
        rotation = mount_pose.pose_matrix(mount_pose.MOUNT_TCP_BASE_POSE)[:3, :3]
        np.testing.assert_allclose(rotation[:, 0], [0, 1, 0], atol=1e-8)
        np.testing.assert_allclose(rotation[:, 1], [0, 0, -1], atol=1e-8)
        np.testing.assert_allclose(rotation[:, 2], [-1, 0, 0], atol=1e-8)
        tip_offset_in_tool = np.array([.09, .05, 0])
        np.testing.assert_allclose(mount_pose.MOUNT_TCP_BASE_POSE[:3] + rotation @ tip_offset_in_tool,
                                   mount_pose.MOUNT_CAD_TIP_BASE_M, atol=1e-8)

    def test_proposed_move_sends_distant_start_to_controller_ik(self):
        script = mount_pose.proposed_move_script(1234)
        self.assertIn("get_inverse_kin_has_solution", script)
        self.assertIn("get_inverse_kin(target, q_start)", script)
        self.assertIn("movej(q_goal, a=0.200000, v=0.100000)", script)
        self.assertNotIn("q_goal[i] - q_start[i]", script)
        args = SimpleNamespace(robot_ip="192.0.2.1", execute=True, path_clear_verified=True)
        robot = MagicMock()
        state = {"actual_TCP_pose": np.array([0, 0, 0.3, 0, 0, 0])}
        with patch.object(mount_pose, "dashboard_info", return_value={"robot_model": "UR3e", "remote_control": "true"}), \
             patch.object(mount_pose, "RTDE", return_value=robot), \
             patch.object(mount_pose, "set_bare_flange_tcp"), \
             patch.object(mount_pose, "bare_joint_state", return_value=(state, np.zeros(6))), \
             patch.object(mount_pose, "stop_running_program", return_value=False), \
             patch.object(mount_pose, "secrets") as secrets, \
             patch.object(mount_pose, "fresh_state", return_value={"actual_qd": np.zeros(6),
                                                                    "output_int_register_25": 1234,
                                                                    "output_int_register_24": -1}), \
             patch.object(mount_pose, "upload_script") as upload:
            secrets.randbelow.return_value = 234
            with self.assertRaisesRegex(RuntimeError, "no inverse-kinematics solution"):
                mount_pose.move_to_proposed(args)
            upload.assert_called_once()


if __name__ == "__main__":
    unittest.main()
