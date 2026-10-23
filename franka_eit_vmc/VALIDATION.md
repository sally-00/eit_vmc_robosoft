# Offline validation — 23 October 2026

No physical robot or EIT sensor was connected or operated during this work.

Completed checks:

- Built the complete ROS 2 package on ROS 2 Jazzy with GCC 13.3, using an isolated official libfranka 0.13.3 build. This compiled both experiment executables, ready motion, generated messages and installed Python tools.
- Started the installed joystick, ball and ready executables with their parameter files and `connect_robot=false`. All parsed successfully, reported the positive-X/Z, Y=0 example ready centre, and exited without any FCI connection.
- Started the installed launch file with a generated mount overlay, again with robot connection disabled; the process exited cleanly.
- Built and ran the Eigen-only core test. It checks point-Jacobian geometry, initial EIT/centre attachment selection, joystick attachment retention, ball progress correction with elastic-extension preservation, a shared centre attachment, rejected mid-trial condition changes, rejected demo/stale sensor input, contact-loss handling, workspace/joint-7/period/heartbeat/wrench guards, bounded stop-torque slew and finite torque output under invalid input.
- Ran both CTest targets in the full ROS build: control core and Python EIT model tests passed.
- Four Python model tests passed: phase-frame parsing/clipping and 40-channel protocol, demo-model rejection flag and contact inference, safe NPZ save/reload, and all adjacent engraved-number midpoint transforms for both CCW and CW plates. The invalid pair [5,5] is rejected.
- Checked all new Python/launch files for syntax errors and exercised installed model-inspection, sensor-node help and mount-configuration tools.
- Verified root Git filtering with representative paths: controller sources, EIT code and exported models are included; paper/simulation files, raw logs, cache and build outputs are excluded.
- Copied the package into an isolated temporary directory and reran model loading/inference tests successfully, without access to the original source modules.
- Audited runtime sources for dependencies on original workspace paths; only self-relative paths remain in tests. The existing contact model and serial reader are copied into the package, not imported from outside it.

Limits:

- ROS 2 Humble was the version used by the inspected VMC Transformer reference package; this package's actual offline build was on Jazzy. Build again against the target computer's compatible ROS 2/libfranka installation.
- The bundled NPZ model is synthetic UI-signal training data, not a physical-pad calibration and not the FEM simulator's learned decoder. Its small example errors are not hardware accuracy evidence.
- The mounting direction is configured as the midpoint between 4 and 5 facing Base +X, using the existing CCW contacting-face convention (67.5° ready pad yaw). A physical model, measured EE-to-pad transform/payload and actual task geometry remain site-specific inputs.
- Control gains, actual sensor rate/latency, workspace reachability, contact forces, collision-free paths, robot-loop timing and experiment success have not been physically validated.
- The q7 guard limits deviation from the captured/saved reference. It is not an exact inverse-kinematics constraint or a guarantee that every configured pose is reachable.
