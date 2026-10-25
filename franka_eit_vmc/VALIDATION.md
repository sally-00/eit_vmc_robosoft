# Offline validation — 24 October 2026

No physical robot or EIT sensor was connected during this update.

## Current workspace checks

- ROS 2 Humble build: `colcon build --packages-select franka_eit_vmc` completed with system Python 3.10. The package installs the measured `models/position_model.npz` checkpoint.
- `colcon test --packages-select franka_eit_vmc`: all three CTest targets passed (control core, EIT inference, operator block helper).
- The bundled checkpoint is a 16-electrode, combined-scan RBF model with 304 electrical channels and 608 real features. Its copied SHA-256 matches the source session: `355b9a75f5e8715863ca6bf17608dc168ae7095c24a8b0188894df48824fa0ac`.
- The ROS runtime and training toolkit produced identical position estimates for all 40 recorded touches in `2026-10-24_11-41-56_JST_d87beb16_david_hydrogel_works/touches.jsonl` (maximum observed difference: 0 mm). All 200 recorded unloaded frames were classified as unfamiliar by the runtime model.
- The six package inference tests check the 304-channel protocol, saved reader settings, frame parsing, direct checkpoint loading, saved RBF predictions, contact persistence, clipping policy, invalid model shapes and engraved-number orientation mapping.

## Limits before hardware use

- The source session's `evaluation.json` reports 2.79 mm median and 21.61 mm maximum position error on 20 held-out touch repetitions at trained locations. It does not establish accuracy at unseen locations, in a later session or during robot motion.
- Contact state now uses three familiar model predictions to engage and two unfamiliar predictions to release. Familiarity is a position-model diagnostic; its use as a contact detector has not been validated on a live sensor. Contact strength was below the old five-times-baseline-noise threshold for 30 of the 40 recorded touch captures, so that rule was removed.
- The trained model permits clipped frames. The saved acquisition pattern and RX gain margin are loaded from its metadata, but actual firmware setup, stream timing, sensor freshness, wiring and live classification still require checks on the connected device.
- Tool transform, payload, control gains, workspace reachability, collision-free paths, physical contact forces and robot-loop timing have not been physically validated. The software workspace and q7 guard are not collision planning.
