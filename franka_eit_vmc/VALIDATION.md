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


## Optional inward-arc mechanism — 25 October 2026

No physical robot, ESP32 or EIT sensor was connected for this change.

- Pulled the latest tracked code at `f6f8e2b` before editing.
- Added the optional full-3D task coupling and four-link common planar-slider alignment, both using the configurable inward depth. Default `legacy` control is retained.
- Core tests cover inward target positions/velocities, pressure direction at alignment, free-slider force balance, guide stops, separate contact/slider Jacobian mappings, summed-force limiting, rejection of bad depth, isolation from legacy contact gains, and rejection/loss of stale, invalid or clipped shaft feedback. Existing core regressions also pass.
- Seven decoder tests cover Base-X sign/channel mapping, ADC rail rejection, source timestamps, filtered rates, opposite ADC polarity, upright overrides and invalid calibration.
- Fifteen block-runner tests use mocked robot/ROS/process calls, including mechanism/depth selection, starting shared angle feedback, startup failure before motion, calibration-range/depth checks and prior trial-order/stop behavior.
- The full ROS 2 Jazzy/libfranka package compiles in the temporary validation workspace, including the new `JoystickState` message, telemetry and executables. All four CTest targets pass: core, EIT inference, shaft feedback, and experiment block.
- An installed-layout ROS smoke check feeds a temporary raw CSV through the actual feedback node and receives `JoystickState` messages. It checks continuous calibrated CSV output, degree/radian conversion, rail invalidation and preservation of old source timestamps. It never opens USB or FCI.

The measured ADC calibration reaches rails at its approximate ±35° stops. New-mechanism blocks reject commanded amplitudes outside that range; 30° is the documented example. Rate filtering and host-receipt timestamps do not identify physical angle accuracy or the full USB/control delay. The real computer needs a rebuild for the new message definitions. Geometry, gains, actual pressure, pad friction, feedback delay and closed-loop hardware performance remain unvalidated by these offline checks.

## Tilted-start two-axis update — 25 October 2026

The recorded x-minus startup had a stable X ADC value near 1139, corresponding to approximately +15.5° Base Y tilt, while its Base X angle stayed near zero. This update accepts stable, nonclipped tilted starts only in `inward_arc`; legacy blocks still require upright. Decoder tests cover the real startup sample, both calibrated channels and rail rejection. Core tests cover the 3D shaft axis/rate, contact-defined task pivot, initial tilted reference, palm normal aligned against the shaft, and Base Y command progressing toward zero while Base X progresses toward the absolute target. Block-runner tests verify tilted starts are allowed for both automatic conditions. The package builds and all four CTest targets pass. Physical robot motion with this two-axis change has not been tested.

## ADC-rail continuation — 25 October 2026

The decoder now keeps the bounded ±35° angle estimate valid at either ADC rail and records `clipped` separately. The block runner accepts a stable rail reading in inward-arc mode, and the controller no longer uses the clip flag as a fault condition. Tests cover rail decoding, automatic start checks, and a fresh clipped controller sample. Stale timestamps, invalid samples and lost feedback still stop inward-arc control. This change has not been physically tested.

## Joystick contact-drop continuation — 25 October 2026

The x-minus baseline trace at 17:37:18 stopped when EIT pattern distance crossed the model support limit despite continued signal. The joystick controller now requires contact for approach engagement but retains the established attachment and continues preload/interaction if the EIT contact bit later drops. Core tests cover both baseline and EIT attachments, inward-arc continuation, and unchanged ball contact-loss behavior. The updated controller has not yet been run on the physical robot.

## Inward-arc parameter validation — 25 October 2026

The live x-plus controller reported `joystick_length=0.020 m`, `active_radius=0.036 m` and `joystick_slider_radius_m=0.050 m`. A requested inward depth of 0.005 m was below the physical length; the old generic validation error actually came from comparing virtual slider travel with the EIT pad active radius. These quantities are independent. The comparison was removed, individual parameter errors now identify the failing field, and a core test covers this exact configuration. Hardware motion with these gains and travel has not been validated.
