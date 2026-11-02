# Offline validation — 24 October 2026

## Direct ball VMC and upward-yielding carriage — 27 October 2026

The hardware workspace source now uses direct horizontal goal/velocity feedback
and the simulation's unilateral normal coupling with upward-yielding contact
brake. Both position estimators share recovery by default; an explicit ablation
keeps the brake engaged after first contact. Initial versus updated attachments
are independently selectable. New normal-state fields are recorded in ROS/CSV.

All 13 existing config files and all previous Config initializer values are
unchanged. Additional parameters are separate; obsolete horizontal-carriage
parameters remain accepted but unused. Initial approach, settling, physical
sensor, hardware gains and protection limits remain different from simulation.
The preserved 10 N goal cap now acts directly on palm drive, not through a 2:1
transmission. Joystick control was not changed.

ROS 2 Jazzy builds successfully in `/tmp/eit-franka-ball-ros-build`, installed
for testing into `/tmp/eit-franka-ball-ros-install`. All ten CTest suites pass,
including the SDK torque-path and recording/operator tests. The normal-mechanism
test matches saved simulation states covering all six responsive recovery runs;
it also checks unilateral force, upward yielding, travel limits and invalid-data
holding. Core tests cover shared EIT/baseline recovery, observed progress,
missing-position handling, and both rolling axes/directions. See
`ball_vmc_validation_20261027.json` and `test/ball_simulation_normal_provenance.json`.

These are offline checks. No robot commands were executed and the active
workspace installation was not replaced. Rebuild the workspace and restart
controller/recording processes together before using the new message schema.


## Experiment reproducibility records — 26 October 2026

Joystick and ball block/standalone operations now save all declared parameters,
per-operation status, parameter-change events, calibrated EIT baseline frames,
actual acquisition gains/protocol, loaded model/calibration bytes, feedback
decoder settings, launch inputs, software environment and SHA-256-addressed
artifacts. Controllers embed their own build-source archive (including local
edits/untracked files and Git/compiler metadata) and expose read-only process,
SDK and startup robot-tool metadata. Required recording failures prevent the
next motion; final failures are retained as incomplete snapshots.

The package builds in `/tmp/eit_recording_build` against ROS 2 Jazzy and
libfranka 0.17.0. All nine CTest targets pass. Isolated localhost ROS tests exercise
the actual operator client for joystick baseline/EIT/kinematic and ball
baseline/EIT, checking archived parameters and baselines, and temporary threshold
changes that are restored before the final dump. Unit checks cover all ROS
parameter types, dirty/untracked build inputs, artifact hash checks, missing-node
failures, physical notes and rejection of foreign/reused process identities.
An independent rclcpp probe confirms startup metadata can be declared while
remaining read-only afterward. A temporary installation in
`/tmp/eit_recording_install` verifies the installed recording tools import and
expose their CLI options without source-checkout lookup.

One test run encountered a ROS discovery timeout while repeatedly restarting an
operator with the same node name; isolated test clients now use unique node names.
These checks used fake controllers/sensors and no hardware. The active workspace
installation and running processes were not replaced; rebuild and restart the
controller, contact node and feedback node before using this recording format.

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
- Tool transform, payload, control gains, workspace reachability, collision-free paths, physical contact forces and robot-loop timing have not been physically validated. The software workspace is not collision planning.


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

## Wrist posture deviation — 25 October 2026

Removed the stop based on joint 7 deviating from its captured/saved posture and removed `q7_tolerance`. The reference posture still supplies the soft null-space preference. Regression coverage checks that positive and negative 0.5 rad wrist deviations are accepted in Hold and Ready, while the actual joint-position limits still stop control. The historical joint7 fault-code slot remains reserved so other numeric fault codes keep their existing meanings. No robot motion was used to verify this change.

## Contact release with shaft snapback — 25 October 2026

In inward-arc mode, combined fresh EIT no-contact and a rapid return near upright while a substantial tilt is still commanded now ends the trial as unsuccessful. The detector enters retraction before calculating another shaft-alignment command. Core regressions cover both signs, a delayed EIT frame, isolated electrical dropout, slow return, near-zero command, completed retraction and clearing the result for the next trial. Block tests verify the unsuccessful outcome is retained while the next condition runs; other controller errors still abort. An isolated localhost ROS test with a fake controller verifies that only a complete release retraction produces the reserved continuation result, that incomplete retraction requests stop, and that normal trials keep their existing result. All six CTest targets passed. The package was rebuilt and installed; no physical robot motion was used to validate this behavior.

## Independent saturating springs — 25 October 2026

The inward-arc task now has one scalar 3D spring stiffness and one scalar damper. Task elasticity and each alignment link saturate independently by vector norm; damping is added separately. The shared resultant-force scale and legacy XY/Z force caps are removed from inward-arc contact control. Slider velocity is solved using the actual saturated spring resultant, retaining zero tangential guide force away from its travel stop. Tests verify radial saturation and energy gradients, saturated slider balance, uncapped damping, independence of task force from alignment saturation, isolation from legacy gains/force caps, and the separate Jacobian mappings. All six CTest targets passed against the new build, including the isolated ROS operator test. The build is staged pending the chosen task/per-link elastic-force saturation values; it has not been installed or tested in robot motion. Existing moment, joint-torque slew, SDK and native limits remain for review.

## Selectable joystick tilt axis — 25 October 2026

`run_block.py --axis x|y` selects the Base direction of shaft tilt for either mechanism. The initial/default axis remains X. A fixed-size single-writer/single-reader queue transfers mechanism, depth and axis together to the realtime thread; changes apply only in Hold/Done. Status acknowledgment includes the actual axis before the block starts motion. Trial CSVs retain both angle commands and the selected axis; plot comparisons select the matching displacement, calibrated ADC channel and palm tilt. Old traces without an axis field default to X.

All seven CTest targets pass against the staged build. Core tests traverse positive and negative commands on both axes for both mechanisms, check the legacy arc and palm-rotation sign, preserve two-axis measured alignment and reject an active-trial axis change. Mocked block tests check axis logging and stop before motion on a mismatch. An isolated localhost ROS fake controller verifies combined selection and axis-only updates, including waiting for delayed status acknowledgment and preserving mechanism/depth. Plot tests use an upright X channel and a distinct +20° Y channel, checking comparisons and all four generated plot types. The updated package is built but remains staged with the independent spring changes; no installation or robot motion was performed.

## Workspace installation — 25 October 2026

The tested independent-spring and selectable-axis build was subsequently installed into `/home/birl/franka_ros2_ws/install/franka_eit_vmc`. The current source configuration was retained, including task elastic saturation at 15 N and the user's alignment elastic saturation at 20 N per link. No physical controller was running during installation and no robot motion was started. Installed executable options, message fields and configuration were checked after installation. Restart controller processes to load the installed build.

## Y-offset landing overlays — 25 October 2026

Added `joystick_y_plus.yaml` (X=0, Y=+0.035 m) and `joystick_y_minus.yaml` (X=0, Y=−0.025 m), mirroring the existing X offsets with the same ready height, workspace minimum and joystick length. The block runner accepts both names and uses its existing two-coordinate configuration check. The overlay does not select the trajectory axis. All 20 block tests pass; additional mocked checks verified both new names reach the pre-motion configuration check and load the expected offsets. Installed both overlays, the updated block runner and documentation. No controller was launched or robot motion started.

## Tool weight compensation and error plots — 25 October 2026

Configured `tool_weight_compensation_mass_kg: 0.5` adds 4.903325 N in Base +Z through the EE translational Jacobian, independently of contact attachments, task-spring caps and alignment forces. Core tests verify the torque difference, zero added moment, force telemetry, normal control stages and bounded stop/fault behavior; invalid masses are rejected. The additional force is recorded explicitly and included in the requested resultant force. This does not set the robot's payload mass, inertia or centre of mass.

Joystick block exit now records `block_result.json` and attempts plotting even if recorder startup fails or cleanup raises an error. The plotter first saves `run_summary.png`, supports empty/missing ESP32 data and ready-only traces, labels partial trials incomplete and omits incomplete CSV rows. Mocked tests cover startup errors, ready errors, Ctrl+C, cleanup failure and optional desktop opening with `--show-plots`. Plot tests verify empty/startup and ready-only summaries and partial-row handling. All seven CTest targets passed, with block and plot targets rerun successfully after the final plot-option changes. The package was rebuilt, tested and installed after the user stopped the active joystick controller. No robot motion was started. Historical run `y_plus_2026-10-25_22-35-37_189673_+0900` was replotted: baseline finished; EIT ended during interaction at approximately 4.8 s.

## Y alignment investigation — 25 October 2026

Offline inspection of the completed 22:42:20 positive-Y run found palm Y tilt approximately 1–3 degrees while the requested palm tilt followed the measured shaft to approximately ±30–35 degrees. Link elastic saturation occurred in approximately 59% of interaction samples in both conditions, with minimum scale 0.26–0.28; neither the 5 Nm moment cap nor slider travel stop was reached. Reconstructing the four elastic links during settled tilted portions gave median uncapped elastic moments approximately 11.6 Nm (baseline) and 9.8 Nm (EIT), reduced to approximately 4.0 Nm by the 20 N per-link caps, matching logged moments. This identifies limited spring authority, but does not establish the physical source of the remaining tracking error.

New core regressions verify positive/negative Y alignment torque signs, the shaft rotation/rate transformation between X and Y, and covariance of saturated alignment forces, moments and slider rates under a 90-degree Base rotation. The core tests pass. An offline model from the local FR3 description matched logged palm normals and positions (position discrepancy about 0.13 mm), and reconstructed joint wrench mapping matched logged torques within the small remaining bias/posture terms. The reconstructed minimum Jacobian singular value was approximately 0.20–0.23, so this check did not support a severe singularity explanation. No controller code, gains, geometry or force limits were changed during this investigation. Diagnostic PNG/JSON files were saved in the run folder.

## Remove duplicate torque slew limiting — 25 October 2026

Removed the package `torque_rate` parameter and its 50 Nm/s limiter. The limiter referenced the SDK-filtered previous torque and was then filtered again by the SDK's 100 Hz filter; at a 1 ms cycle this reduced saturated slew to approximately 19.3 Nm/s. Offline reconstruction of failed runs showed large wrench-request versus torque-command differences during growing oscillation. Normal torque requests now go directly to libfranka; its 100 Hz filter and native approximately 1000 Nm/s torque-rate protection remain enabled explicitly. Stop/fault requests zero and waits for the SDK-applied desired torque to fall below the existing completion tolerance before finishing control.

All eight CTest targets pass. The new SDK torque-path test uses libfranka's actual low-pass and rate-limit functions to verify step/reversal response, native per-cycle bounds and stop/fault completion only after applied torque reaches zero. Core regressions confirm there is no package slew on normal torque and that nonfinite input remains rejected. Installed files were replaced atomically so any existing controller keeps its loaded code until restart. No robot motion or controller stop was initiated by the assistant; physical stability with the revised command path remains untested.

## Kinematic contact-position comparison — 25 October 2026

`run_block.py --include-true-position` adds a third `kinematic` condition. The controller computes the physical tip from the measured two-axis shaft direction, configured pivot and physical length, projects it into the measured palm plane at confirmed contact, and retains that material attachment throughout manipulation. This matches the existing frozen EIT attachment policy. EIT still supplies contact detection and all conditions retain the same ready/baseline procedure and mechanism. Continuous measured tip coordinates and the canonical contact source are logged separately.

Core tests cover both mechanisms, a yawed palm with a separate EE-to-pad rotation, independence from invalid EIT position estimates, capture-once behavior, updated tip diagnostics, stale shaft rejection, out-of-pad rejection and rejection for the ball task. Mocked block tests verify three recorded outcomes and all six condition orders. Isolated localhost ROS tests verify condition acknowledgment and the kinematic selector through the actual experiment client. Plot tests render all three conditions and retain old two-condition behavior. The eight test targets pass, including the updated plot fixture; core/SDK tests were rerun after formatting/comment changes. The build is installed with updated messages, scripts and documentation. No physical robot motion was used to validate it. The geometry reference is not an independently measured contact-position ground truth.

## Signed ball motion on X or Y — 26 October 2026

Added `ball_axis: x|y`, `initial_ball_y` and signed `roll_distance` support throughout the carriage goal, EIT/robot-motion observations, continuity-preserving reference rebasing, force transmission and logging. Defaults retain +X rolling by 20 mm. The separate `ball_y_minus_10cm.yaml` overlay requests Y=−0.10 to −0.20 m with a 30 s horizon and extends the software Y minimum to −0.32 m for the nominal 20 cm palm stroke. The user expects slip and accepted the contact-area limitation of the 90 mm pad; the overlay does not guarantee continued contact or physical goal attainment.

Core tests cover both Base axes, both distance signs and both observation conditions using a synthetic no-slip contact measurement. Workspace validation tests reject both an out-of-bounds ball goal and an in-bounds ball goal whose doubled nominal palm stroke exceeds the workspace. All nine CTest targets pass, including recording/provenance tests. Ball goal messages and CSVs retain explicit X/Y goals plus the selected scalar goal. The updated package and example overlay are installed; no robot motion was initiated by the assistant. Physical reachability, clearance, slip and actual rolling performance remain untested by these offline checks.

## Quote ROS axis parameters — 26 October 2026

The native ROS YAML parser interpreted unquoted `ball_axis: y` as a boolean and rejected controller startup. Quoted all axis values in the shipped ball/joystick YAML files and installed the corrected configurations. A native rclpy-node test loads the same controller/task/overlay parameter-file chain and declares each axis as a string, verifying ball `'y'` and joystick `'x'` without importing or connecting to robot hardware. This covers a difference missed by the earlier PyYAML configuration check. No controller rebuild or robot motion was required for the configuration correction.
