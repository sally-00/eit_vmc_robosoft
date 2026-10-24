# Offline validation — 24 October 2026

No physical robot or EIT sensor was connected or operated during this work.

Completed checks:

- Built the complete ROS 2 package on ROS 2 Jazzy with GCC 13.3, using an isolated official libfranka 0.13.3 build. This compiled both experiment executables, ready motion, generated messages and installed Python tools.
- Built and ran the Eigen-only core test. It checks point-Jacobian geometry, initial EIT/centre attachment selection, joystick attachment retention, ball progress correction with elastic-extension preservation, a shared centre attachment, rejected mid-trial condition changes, rejected demo/stale sensor input, contact-loss handling, workspace/joint-7/period/heartbeat/wrench guards, bounded stop-torque slew and finite torque output under invalid input.
- Ran both CTest targets in the full ROS build: control core and Python EIT model tests passed.
- Five Python tests passed: phase-frame parsing/clipping and 40-channel protocol; demo-model rejection flag, inference hysteresis and predicted units; all adjacent engraved-number midpoint transforms; loading/prediction with regression fitting explicitly disabled; and rejection of unexported, malformed, nonfinite and object-array models. The invalid pair [5,5] is rejected.
- Checked all new Python/launch files for syntax errors and exercised installed model-inspection, sensor-node help and mount-configuration tools.
- Verified root Git filtering with representative paths: controller sources, EIT code and exported models are included; paper/simulation files, raw logs, cache and build outputs are excluded.
- Copied the package into an isolated temporary directory and reran model loading/inference tests successfully, without access to the original source modules.
- Audited runtime sources for dependencies on original workspace paths; only self-relative paths remain in tests. The existing contact model and serial reader are copied into the package, not imported from outside it.

Inference-only update checks (24 October):

- Removed collection, training, synthetic-model generation and model-saving code from the ROS package. A fresh ROS build/install contains no collector or trainer entry points. Both CTest targets pass, and installed model inspection and sensor-node help run successfully.
- The loader reads precomputed RBF weights, width and support limit. Bundled-model predictions match the previous decoder at all 60 stored feature vectors to absolute/relative tolerance 1e-12.
- The separate `export_franka_model.py` tool, also included in `eit_position_toolkit/`, reproduces the previous RBF parameters and leaves the input checkpoint unchanged. It is not installed or imported by the ROS package. Existing checkpoints must be exported before deployment; the current runtime supports all 40 adjacent-scan channels and RBF models only.
- Unloaded baseline acquisition remains a runtime electrical-reference measurement; it neither fits nor writes model parameters.

Physical experiment workflow update (24 October):

- Startup now connects to FCI after configuration validation; the previous configuration-only execution branch and launch toggle were removed. Robot executables were not started during this update.
- Rebuilt the ROS package and passed all three CTest targets (core, inference, and four mocked operator-block checks). The installed block helper help command runs without starting control.
- Base-X landing presets are 0 and ±10 mm, with Base Y=0. An operator-paced block helper checks the running controller offsets and ball attachment policy, alternates condition order and requires object reset/recording prompts. The README specifies the ordered 30 joystick and 20 ball trials.

Limits:

- ROS 2 Humble was the version used by the inspected VMC Transformer reference package; this package's actual offline build was on Jazzy. Build again against the target computer's compatible ROS 2/libfranka installation.
- The bundled NPZ model is synthetic UI-signal training data, not a physical-pad calibration and not the FEM simulator's learned decoder. Its small example errors are not hardware accuracy evidence.
- The mounting direction is configured as the midpoint between 4 and 5 facing Base +X, using the existing CCW contacting-face convention (67.5° ready pad yaw). A physical model, measured EE-to-pad transform/payload and actual task geometry remain site-specific inputs.
- Control gains, actual sensor rate/latency, workspace reachability, contact forces, collision-free paths, robot-loop timing and experiment success have not been physically validated.
- The q7 guard limits deviation from the captured/saved reference. It is not an exact inverse-kinematics constraint or a guarantee that every configured pose is reachable.
