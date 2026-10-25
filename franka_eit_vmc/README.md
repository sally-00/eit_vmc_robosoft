# Franka EIT–VMC proof-of-concept package

Two ROS 2 executables implement the matched joystick and ball-rolling experiments:

- `joystick_controller`: centre attachment versus EIT-defined attachment at contact.
- `ball_roll_controller`: robot-motion versus EIT position feedback, with centre attachment shared.
- `move_ready`: move the active pad face above the task at positive Base X/Z and Base Y=0.

The package is self-contained with respect to research code: it contains the controller mathematics, Amodo ARIA serial reader, position decoder, inference node, measured model, and configuration. Runtime dependencies are ROS 2, Eigen, a compatible libfranka installation, NumPy, PyYAML and pyserial. There are no imports, symlinks or absolute paths back to the original project.

**Model selection:** The EIT node and pad viewer select the most recently modified `position_model.npz` or `position_model_mlp.npz` under `eit_position_toolkit/contact_position_runs/*/` when they start. The selected file path and SHA-256 are printed. The ROS decoder currently supports trained RBF checkpoints; if the newest file is an unsupported MLP checkpoint, loading stops with an error. Use `--model PATH` to select a specific checkpoint. The ROS package does not collect touches or train a model.

The controllers have not been run on a physical robot. See `VALIDATION.md` for the checks actually performed.

## Robot interface and the VMC Transformer reference

The existing VMC Transformer FR3 package was inspected. This package follows its Base-frame convention, libfranka EE-frame tool offsets, Jacobian-transpose torque control, Coriolis compensation, and captured-posture convention. It does not depend on or modify that package.

These executables use **direct libfranka FCI torque control inside ROS 2 nodes**, not a `controller_manager` plugin. Stop `franka_bringup`/other FCI drivers before connecting one of these executables. Only one experiment or ready-motion executable may own the robot at a time. ROS sensor, recording, operator and external camera nodes can run alongside it. The package never disables Franka's native collision/reflex checks or automatically recovers robot errors.

Use the libfranka version already compatible with the robot's installed system. Do not replace the robot computer's SDK solely because a different version was used for an offline compile check.

## What is controlled

All lengths are metres, angles radians, forces N, and torques Nm. The configured pad frame is centred at the active contacting face, with local +Z pointing out of that face toward the object. Its XY coordinates match the model's engraved-face view.

For a local pad contact `c`, the point in libfranka EE coordinates is

```
r_EE = ee_to_pad + ee_R_pad * [c.x, c.y, 0]
p_c  = p_EE + R_EE * r_EE
J_c  = J_EE_linear - skew(R_EE * r_EE) * J_EE_angular
```

The torque loop reads robot pose, joints, velocity, Jacobian and Coriolis directly through libfranka. It runs the Cartesian coupling, normal preload and orientation controller, then applies

```
tau = J_c^T * force + J_angular^T * moment + null_space_posture + coriolis
```

Franka's torque interface already compensates gravity; the package does not add gravity again. A damped null-space projector retains the captured/saved posture. Joint torque commands have no package magnitude cap; they are slewed from the previous hardware command at the configured Nm/s rate. Workspace, point/joint speed, joint margin, update-period, operator heartbeat and EIT freshness guards stop the trial. The software workspace bounds the EE, pad centre, selected point and reference; it is not full-arm collision checking. Native Franka collision/reflex monitoring remains in force.

The update-period guard allows up to `max_period: 0.010` s during motion and `idle_max_period: 0.020` s while holding or done. The larger idle allowance avoids aborting a completed trial for one tolerated callback gap while leaving the active-motion guard unchanged. A period fault log prints the observed callback interval, limit and mode; libfranka may also stop independently for communication problems.

### Joystick

At electrical contact, the condition chooses the pad centre or the inferred EIT location. This attachment remains fixed in pad coordinates throughout manipulation. The commanded angle `theta_d` drives both the geometric arc and the desired palm orientation:

```
h(theta_d) = h0 + [L*sin(theta_d), 0, L*(cos(theta_d)-1)]
R_pad_desired = rotation_about_Base_Y(theta_d) * R_pad_ready
```

Set the commanded amplitude in `config/joystick.yaml` with `joystick_angle_deg` (currently 45.0); the controller converts it to radians internally and requests +angle, then -angle. The value must be finite and positive; there is no additional command-angle cap in this package. The angle reference has bounded speed and acceleration. `L` is the supplied pivot-to-contact radius (`joystick_length`), shared by both conditions. Actual joystick angle is **not** supplied to the controller. Measure it independently for evaluation.

Horizontal spring–damper forces act at the selected point. A separate normal spring–damper ramps a nominal downward preload (example 3 N), and orientation control follows `theta_d`. Preload is an impedance bias, not force feedback from EIT. Both conditions receive the same contact detector, trajectory, gains, supplied geometry and limits.

### Ball rolling

Both proof-of-concept conditions use the **pad centre attachment** (`update_ball_attachment: false`). Only the progress observation changes:

```
robot-motion observation = independently measured initial ball X + 0.5 * palm travel X
EIT observation          = Base X of the sensed pad contact
```

The EIT observation approximates ball-centre X for a near-horizontal pad. The controller rejects excessive tilt. Both observations are filtered and applied on fresh EIT electrical frames, using the same correction rule. Contact state is shared.

The carriage is a software state, not the physical ball:

```
h(s) = h0 + 2 * [1,0,0] * (s-s0)
M_v * s_ddot = bounded(k_g * (s_g-s)) - d_v*s_dot - G^T*f_c
G = 2*[1,0,0]
```

The 2:1 ratio approximates rolling between a stationary table and a translating palm. Physical slip is allowed. The same coupling force acts on the robot and, through the transmission, on the carriage; it is not a measured EIT force. The fixed goal `s_g` is `initial_ball_x + roll_distance`. Supply `initial_ball_x` from the same independent measurement for both conditions; it is not silently initialized with EIT in the baseline.

At each fresh observation, `s` is corrected by `clip(0.20*(filtered_observation-s), ±0.003)`. `h0.x` is shifted by `-2*correction` to preserve elastic spring extension. This does not establish full force/torque continuity or passivity.

Optional attachment updates exist for a later migration experiment. They filter/bound the contact change and rebase the reference and normal anchor. Leave them disabled for the two-condition ball proof of concept.

### Trial states and stopping

Each experiment starts holding its current pad pose. `move_ready` lifts to the ready height, translates/orients above the task, and descends to the configured ready height. This is a bounded local Cartesian route, not a collision-aware motion plan. The starting EE and active pad face must already lie inside the configured workspace. A trial requires the robot to be at that ready pose and the pad to be electrically unloaded.

The positioning spring/damper force during hold, ready movement, approach and retract has no combined Cartesian force cap. Trial normal force retains `normal_force_limit` and trial horizontal force retains `force_xy_limit`. These are controller command limits, not measured contact-force limits. Joint speed and workspace guards remain active. The controller no longer stops solely because the moving reference is far from the measured pad position; it still requires the pad to be at the ready pose before starting a trial.

Ready movement and retraction use `ready_k: 3000` N/m and `ready_d: 60` N·s/m for Cartesian position control. Holding and approach use `hold_k: 600` N/m and `hold_d: 35` N·s/m. The ready move uses `ready_kr: 90` Nm/rad, `ready_dr: 5` Nm·s/rad, and a 4 Nm orientation command limit. Approach and preload use their own `approach_kr: 90`, `approach_dr: 5`, and `approach_moment_limit: 4` to resist pad rotation while establishing contact. Manipulation uses the compliant `orientation_k: 3`, `orientation_d: 1`, and 1 Nm `moment_limit`; retraction retains `hold_kr: 30`, `hold_dr: 3`, and that 1 Nm limit. The configured torque-rate limit still bounds changes in output torque. The ready route advances after the pad settles within 5 mm of each waypoint and meets the orientation/velocity checks; trial start also accepts up to 5 mm.

The package does not apply its own stop threshold to Franka's estimated external force or moment. Status and ready/trial CSVs retain both estimates for inspection. They also record the controller's requested Cartesian force and moment and the final commanded joint torques. The requested Cartesian values precede Jacobian mapping and torque-rate limiting, so they are not measured contact forces. Franka's native collision/reflex behavior remains configured on the robot.

`start_trial` approaches downward. At the first valid electrical contact report, it slows to `approach_contact_speed` and commands another `approach_contact_travel_m` (defaults 5 mm/s and 1 mm). Preload begins after EIT contact remains valid for `approach_contact_confirm_s` (default 0.25 s) across at least two frames and the verification descent completes. The pad may move down or meet resistance; both responses count. If EIT contact disappears, normal approach resumes. The total approach distance and time limits still apply. These signals support contact establishment but do not independently prove physical contact. Contact loss during preload or interaction stops the trial. At the common time horizon the robot retracts upward and holds. If retraction times out but the EIT stream has reported no contact continuously for at least 0.5 s, the controller holds its measured pose and reports `done` with `retract_incomplete=true`; the next block repeat still requires the operator to type `ready` or `r`. A stale stream or continued contact still produces `retract_timeout`. `done` means the commanded manipulation finished; it does not mean the object met externally measured task-success criteria. The CSV records whether retraction was incomplete. Check the physical route before the next ready move.

During preload, the horizontal contact-point spring is active immediately while the downward preload ramps. The orientation target remains continuous with approach and the joystick angle is commanded relative to the measured ready orientation. This avoids the orientation-command jump and weak initial lateral support visible in the failed x_plus traces.

Stop/fault handling slews applied torque toward zero before ending FCI when the callback remains operational. Franka native reflexes and communication failures can terminate control independently. A fault requires restarting the executable; there is no automatic error recovery.

## Build in the current workspace

This checkout is at `~/franka_ros2_ws/src/eit_vmc_robosoft/franka_eit_vmc`. The package can also be placed in another ROS 2 workspace `src/` directory; the rest of the paper/simulation directories are unnecessary.

ROS Humble here uses system Python 3.10. Deactivate Conda before running ROS tools (`conda deactivate` until `(base)` is gone). In this workspace, a prior CMake cache selected Conda Python 3.14 and failed while generating messages with `ModuleNotFoundError: No module named 'em'`. Explicitly select `/usr/bin/python3` when building so an old cache cannot select Conda again. The same Python must be first on `PATH` for scripts using `#!/usr/bin/env python3`.

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
PATH=/usr/bin:/bin:/opt/ros/humble/bin colcon build --packages-select franka_eit_vmc \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

For the whole workspace, omit `--packages-select franka_eit_vmc` from that build command. On 2026-10-24, all 41 packages built successfully with the system Python selected. Colcon emitted some nonfatal CMake and package-override warnings.

The direct-FCI interface avoids coupling to a particular `franka_semantic_components` API. Use a realtime-capable host and compatible FCI configuration as for the existing VMC Transformer hardware experiments.

When updating an older installation of this package, build in a clean workspace so removed training executables are not left in the install directory.

Edit configuration in the package source directory. Set this path in every terminal used below:

```bash
export EIT_CONFIG_DIR=$HOME/franka_ros2_ws/src/eit_vmc_robosoft/franka_eit_vmc/config
```

The package installs `config/` and `models/` during `colcon build`. Rebuild `franka_eit_vmc` after editing source configuration, because the launch file also reads installed task defaults. The launch commands below pass the editable controller and experiment overlays explicitly. Keep trial output outside the package.

Set the actual robot IP and **measured** EE-to-active-face translation/rotation in `controller.yaml`. The supplied EE-to-pad translation is `[0, 0, 0.050]` m, based on the corrected 50 mm measurement. The user confirmed the quarter-turn assembly clocking (pad +X toward EE -Y), and `tool_transform_confirmed` is set accordingly. Measure tool mass/TCP and configure the Franka robot appropriately using your established procedure. The package does not guess payload or change collision thresholds.

The example workspace is positive Base X/Z, centred at Base Y=0. Ready face centre is `[0.45, 0.0, 0.35]`. Adjust its height to be a small measured clearance above the actual object so the 20 mm approach budget can reach it. `landing_offset_x_m` offsets joystick landings along Base X for paired tests; keep `landing_offset_y_m: 0.0` so the task stays centred at Y=0.

FR3 joint limits are defaults. For a Panda, supply `config/panda.yaml` as an overlay and verify the actual robot model/limits.

## Engraved numbers and pad orientation

The midpoint between engraved numbers **4 and 5 faces Base +X**. The default configuration uses this mounting direction: `ready_pad_yaw_rad: 1.1780972450961724` (67.5°) and `mount_orientation_confirmed: true`.

The confirmed symmetric assembly clocking puts pad +X toward EE -Y (`ee_R_pad_xyzw: [0, 0, -0.70710678, 0.70710678]`). This selects a downward-facing EE ready yaw of -22.5° while retaining the 4–5 midpoint toward Base +X. An offline FR3 IK comparison at the joystick target, assuming a 47 mm flange-to-contact offset, gave q7 about +0.20 rad for this clocking versus -0.87, -2.15, or +1.40 rad for the other three. The reference physical VMC run had q7 about -0.03 rad. The axial offset was later corrected to 50 mm; the offline IK result does not verify the physical motion path.

For the existing CCW plate, viewed from the contacting face: 1 is bottom, 3 right, 5 top, 7 left. The shipped `config/mount.yaml` records the 4–5 direction. To regenerate that overlay:

```bash
ros2 run franka_eit_vmc configure_mount.py --pair 4 5 --numbering ccw \
  --view contacting-face --output $EIT_CONFIG_DIR/mount.yaml
```

The helper computes a downward-facing pad rotation whose selected midpoint points Base +X. It does not measure `ee_R_pad_xyzw`. That extrinsic rotation is the physical relationship between the pad frame and libfranka EE. Avoid applying the same mounting rotation in both quantities.

## Optional saved joint posture

The ready command already moves the pad to `ready_pad_position_m` and `ready_pad_yaw_rad` in `controller.yaml`. It follows a bounded lift, translate/rotate, descend route from the current pose. It does not solve for a preferred seven-joint configuration or plan around obstacles. The starting pose and local route still need to be safe, inside the configured workspace, and reachable.

Because the arm has seven joints for a six-dimensional pad pose, it can reach a pad pose with more than one joint posture. The null-space term holds a reference posture while the Cartesian controller moves the pad. By default, `use_saved_posture: false` captures the joints at controller activation; no posture file or capture step is required. The q7 guard allows up to `q7_tolerance` (default 0.35 rad) from that reference. If this prevents a reachable ready move, inspect the start posture and target instead of bypassing the guard without validation.

Only if a specific, previously validated joint configuration must be retained across controller restarts, capture it while the previous controller/driver is at that posture:

```bash
ros2 run franka_eit_vmc capture_posture.py --topic /joint_states --prefix fr3_ \
  --output $EIT_CONFIG_DIR/posture.yaml
```

Stop that FCI driver before running this package. If using the saved posture, add `posture:=$EIT_CONFIG_DIR/posture.yaml` to the launch command. The saved q7 must be near the robot's actual starting q7 or the q7 guard will stop control. A saved posture is a null-space preference, not a command to move to those seven joint angles. The package cannot guarantee that an arbitrary Cartesian pose is reachable near a chosen q7.

## Physical EIT model and inference

The RBF model is a kernel-ridge position decoder on normalized complex-voltage changes. It uses the selected checkpoint's scan settings, estimates pad XY in millimetres internally, and publishes metres. The runtime reconstructs the RBF weights, width and support limit from the checkpoint's saved training vectors and labels, using the same calculation as the training toolkit. Force labels/force predictions are not used.

The source session is `2026-10-24_11-41-56_JST_d87beb16_david_hydrogel_works` in the parent `eit_vmc_robosoft` directory. Its `evaluation.json` reports 2.79 mm median and 21.61 mm maximum error on 20 held-out touches at previously trained locations. These results do not measure error at unseen locations, in later sessions or during robot motion.

To inspect the newest checkpoint without connecting the sensor or robot:

```bash
ros2 run franka_eit_vmc inspect_model.py
```

The loader uses `allow_pickle=False` and validates the model's saved scan shape and electrode protocol. `config/eit.json` provides the serial port and may specify sensor settings; any specified scan, pin, frequency, period or RX gain margin must match the checkpoint. The node fills omitted settings from the checkpoint. Confirm the saved pin order against the physical wiring before connecting the pad.

Before starting inference, power the EIT board with its 24 V adapter, connect its **USB-HS** data port, and identify the serial device:

```bash
/usr/bin/python3 -m serial.tools.list_ports -v
ls -l /dev/serial/by-id/  # if present, these names remain stable across reconnects
```

With `eit_port: null` in `config/eit.json`, the reader selects the single USB serial device with VID:PID `0483:5740`. If it is absent, check power, the USB data cable and port, and `lsusb`; changing permissions cannot create a missing `/dev/tty*` device. If there are multiple matching devices, set `eit_port` in `eit.json` to the chosen `/dev/serial/by-id/...` or `/dev/ttyACM*` path.

If the device exists but opening it says `Permission denied`, grant this account serial access, then **log out and back in**:

```bash
sudo usermod -aG dialout "$USER"
```

For access in the current session, use the EIT port you found above (replace `/dev/ttyACM0` if it has another name):

```bash
sudo setfacl -m u:"$USER":rw /dev/ttyACM0
```

In the new login session, verify access and the detected port:

```bash
id -nG  # must include dialout
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

One `dialout` membership covers both the EIT and ESP32 serial ports when their device nodes are group-owned by `dialout`. Close other programs reading the same port, including Arduino Serial Monitor, before launching the EIT node.

Start live inference:

```bash
ros2 run franka_eit_vmc eit_contact_node.py
```

For standalone sensor checks, request 20 fresh baseline frames with the pad unloaded. This measures the live electrical reference for prediction; it does not train or modify the model. The service returns after calibration completes or reports failure. The experiment ready action calls it automatically at each settled ready pose, so `run_block.py` does not need a separate manual calibration step:

```bash
ros2 service call /eit_contact/calibrate_baseline std_srvs/srv/Trigger '{}'
```

The node publishes `/eit/contact`. Three consecutive familiar predictions indicate contact; two unfamiliar predictions indicate loss. `contact_threshold` in `eit.json` is the startup signal-strength cutoff. Its default `0.0` preserves the original contact decision, including the existing near-zero signal guard. A positive value requires the signal strength to reach that value as well as match a familiar position pattern. The viewer shows the live strength and active cutoff and has a slider that updates the running node's `contact_threshold` ROS parameter without a restart. The slider range expands with observed signal strength; its status line confirms whether the node accepted the update. Slider changes last until the EIT node restarts; edit `eit.json` to make a chosen threshold the startup default. The strength is dimensionless: the Euclidean norm across usable channels of each complex voltage change divided by that channel's unloaded baseline voltage. To choose a cutoff, first recalibrate with the pad unloaded, then compare several seconds of unloaded or near-contact readings with several stable, deliberate touches in the viewer. Set the cutoff above the unloaded/near-contact range and below the stable-touch range only if those ranges separate; if they overlap, a strength cutoff alone cannot classify contact reliably. Positions outside the model support have `position_valid=false`. The selected checkpoint's clipping policy applies; malformed frames remain invalid. Familiarity is a position-model diagnostic, not a separately validated contact detector. Check unloaded and known-touch behavior on the actual sensor before robot trials. The controller rejects nonphysical models, invalid frames, wrong pad frames/engraving conventions, stale timestamps, and invalid positions when required by the selected condition. There is no fixed model SHA-256 in `controller.yaml`; check the path and SHA-256 printed by the node before trials.

The firmware provides no device acquisition timestamp. Published timestamps are host receipt times; software freshness checks do not measure the full electrical acquisition/filter latency. Both processes should use the same host/system clock. On a serial error, including excessive backlog or a busy port, the node closes its handle and retries every 2 seconds. It clears contact state and requires a new unloaded baseline after reconnection. If another process owns the exclusive lock, close that process before retry can succeed. Actual frame rate must be measured on the device; no 20 Hz hardware rate is claimed.

## Run the experiments in order

These commands operate the physical robot. Launching a task connects through FCI and holds the current pose; a ready/trial request starts motion. `task:=ready` starts its finite ready move immediately. All robot examples below use real control. Keep one FCI owner. In every terminal, source the parent workspace and then this checkout's overlay so the executables and shared libraries come from the same build:

```bash
source ~/franka_ros2_ws/install/setup.bash
source ~/franka_ros2_ws/src/eit_vmc_robosoft/install/local_setup.bash
```

### 1. Prepare the hardware and files once

Build using the instructions above and edit the files in `$EIT_CONFIG_DIR/`. Set the real robot IP, measured EE-to-pad transform/payload, reachable task height, workspace and joystick length. Set `tool_transform_confirmed: true` after entering the measured transform. The 4–5 midpoint mounting direction is already configured toward Base +X. A saved wrist posture is optional; the default captures the starting joints automatically.

Review the selected model's source session diagnostics and test known positions on the live sensor. Its checkpoint supplies the pin order, scan, frequency and RX gain margin; confirm those against the connected pad. Do not infer wiring from engraved landmark numbers.

The ±10 mm X offsets below are example pad-centre displacements. Confirm all three landings stay inside the calibrated sensing region with the real joystick. Base Y stays zero. The block helper checks that the launched offset matches its label. Keep the joystick fixture fixed while changing the palm landing.

The joystick overlays assume the fixture base bottom is at robot Base Z=0 and its upright top is at Z=55 mm. They place the ready pad face at Z=75 mm (20 mm clearance) and lower the software workspace minimum to Z=30 mm, leaving the workspace guard active. Joystick approach distance is 30 mm, preserving the previous lowest commanded height of Z=45 mm; its timeout is 10 s. These coordinates do not establish physical reachability or table clearance. Verify the robot's actual Base frame, pad pose, measured EE-to-pad transform, fixture location, and the entire ready/interaction path before motion. The `joystick_length` parameter is set to 20 mm, assuming that is the pivot-to-contact distance; confirm it is not merely the exposed stem length.

### 2. Start EIT inference (terminal A; leave running)

```bash
ros2 run franka_eit_vmc inspect_model.py
ros2 run franka_eit_vmc eit_contact_node.py
```

For an optional standalone sensor check, use terminal C to request a baseline with the pad unloaded. Each `run_block.py` ready action recalibrates automatically after the robot settles at the ready pose:

```bash
ros2 service call /eit_contact/calibrate_baseline std_srvs/srv/Trigger '{}'
ros2 topic echo /eit/contact --once
```

The service accepts recalibration even if the previous contact estimate still says `true`; physically unload the pad before calling it or running ready. The node clears the previous contact state and collects 20 fresh frames. The ready action waits for a subsequent fresh message with `measurement_valid: true`, `model_is_physical: true`, and `contact: false` before a trial can start. For a standalone check, inspect those fields after calibration. When making a known touch, verify contact and position become valid, then release. Before robot trials check that acquisition gaps stay below `sensor_timeout` (default 0.30 s).

For a live pad view during trials, run this in another terminal (with Conda deactivated):

```bash
source ~/franka_ros2_ws/install/setup.bash
source ~/franka_ros2_ws/src/eit_vmc_robosoft/install/local_setup.bash
ros2 run franka_eit_vmc eit_pad_viewer.py
```

The window reads `/eit/contact` only. It reports a model mismatch if the node and viewer loaded different checkpoints. Its dot is the estimated location in the pad contacting-face frame, with X and Y shown in millimetres. It shows live signal strength and the active contact threshold to six decimal places. Positive strength alone does not imply contact: the model must also accept the pattern and predicted position, and the event counters must settle. It shows no dot when contact is absent or the position is invalid, and marks the stream stale after 0.30 s without a message. The display needs a graphical desktop and Tkinter (`python3-tk`).

### 3. Joystick centre block: 10 trials

The ESP32 is plugged into the **same computer** that runs ROS. Close Arduino Serial Monitor, then grant access to its `/dev/ttyUSB0` port before starting the block:

```bash
sudo setfacl -m u:"$USER":rw /dev/ttyUSB0
```

`run_block.py` starts the ESP32 recorder automatically, waits for a sample before any robot motion, and saves `joystick_esp32.csv` beside `block.json` in a dated `center_*` folder. It stops the recorder when the block ends. The robot controller does not read the ESP32. The ESP32 values are raw ADC counts; measure centre and known positive/negative angles to convert them to degrees. In terminal B:

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=joystick \
  config:=$EIT_CONFIG_DIR/controller.yaml \
  overrides:=$EIT_CONFIG_DIR/joystick_center.yaml
```

In terminal C:

```bash
ros2 run franka_eit_vmc run_block.py --task joystick --block center --repeats 5 --output ~/eit_trials
```

Joystick blocks now run automatically with this command: the controller moves to ready, approaches, preloads, executes +`joystick_angle_deg` then −`joystick_angle_deg`, retracts and continues to the next condition without `r` prompts or a plotting pause between trials. The script checks the live ESP32 recording before each ready and trial command and waits up to 20 s for a fresh, stable upright reading near X=3190 and Y=3095 raw ADC counts (within 80 counts on each axis). These defaults reflect this joystick's observed centre; adjust them with `--center-x-raw`, `--center-y-raw`, and `--center-tolerance-raw` if the hardware changes. If it does not return upright, the block stops instead of starting the next trial from a displaced joystick. A controller fault or lost ESP32 stream also stops the block. At the end, the script requests a controller stop, saves the ESP32 recording, and generates the analysis and comparison plots. Use `--manual` to restore the old ready/run prompts. One contact location is used throughout each run. The EIT attachment is defined at contact and stays fixed in pad coordinates. The baseline assumes the pad centre. Both get the same contact detector, gains, length and angle commands. Verify the contact stays approximately fixed on the pad.

Five paired repeats run both conditions, alternating baseline/EIT order across repeats. Automatic mode relies on the joystick returning upright by itself; it cannot physically reset the object. In manual mode, `q` at a prompt exits the block while the task controller continues holding; Ctrl+C during a child operation requests its bounded stop and ends the block. If auto-detection picks the wrong port, pass `--joystick-port /dev/ttyUSB0` using the actual ESP32 port.

### 4. Joystick negative-X block: 10 trials

After the centre block, stop the joystick controller in terminal B and restart it with the negative-X overlay. `run_block.py` reads the landing offsets from `joystick_x_minus.yaml`, checks that both active controller offsets match, and records the active values in `block.json`:

```bash
export EIT_CONFIG_DIR=$HOME/franka_ros2_ws/src/eit_vmc_robosoft/franka_eit_vmc/config
ros2 launch franka_eit_vmc controller.launch.py task:=joystick \
  config:=$EIT_CONFIG_DIR/controller.yaml \
  overrides:=$EIT_CONFIG_DIR/joystick_x_minus.yaml
```

Then in terminal C:

```bash
ros2 run franka_eit_vmc run_block.py --task joystick --block x_minus --repeats 5 --output ~/eit_trials
```

### 5. Joystick positive-X block: 10 trials

Stop the controller in terminal B and restart:

```bash
export EIT_CONFIG_DIR=$HOME/franka_ros2_ws/src/eit_vmc_robosoft/franka_eit_vmc/config
ros2 launch franka_eit_vmc controller.launch.py task:=joystick \
  config:=$EIT_CONFIG_DIR/controller.yaml \
  overrides:=$EIT_CONFIG_DIR/joystick_x_plus.yaml
```

Then in terminal C:

```bash
ros2 run franka_eit_vmc run_block.py --task joystick --block x_plus --repeats 5 --output ~/eit_trials
```

Joystick total: **3 landings × 2 conditions × 5 repeats = 30 trials**. Measure actual angle error over the common command sequence, final angle error, contact losses and completion; measure peak load only with a reliable independent measurement or clearly label the robot wrench estimate. Expect the fixed-centre baseline to degrade more at off-centre landings; centred performance may be similar. These are hypotheses, not established hardware results.

### 6. Ball preparation and undisturbed block: 10 trials

Stop the joystick controller. Install the ball/table setup at Base Y = −0.10 m. `ball.yaml` places the ready pad face at `[0.45, -0.10, 0.35]` m; verify that its Z height and the full approach path clear the actual ball/table setup. Mark the ball's starting position and measure its Base-X centre independently. Set `initial_ball_x` in `$EIT_CONFIG_DIR/ball.yaml`; this same value is used for both conditions. Reset the ball to that mark before every trial. The fixed goal is this initial X plus `roll_distance` (default +20 mm), so the commanded roll direction is **Base +X**, from X=0.45 m toward X=0.47 m with the current values. Leave both landing offsets zero in `controller.yaml`.

Start independent camera recording of ball position; it is for evaluation, not controller feedback. In terminal B:

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=ball \
  config:=$EIT_CONFIG_DIR/controller.yaml \
  overrides:=$EIT_CONFIG_DIR/ball.yaml
```

In terminal C:

```bash
ros2 run franka_eit_vmc run_block.py --task ball --block undisturbed --repeats 5 --output ~/eit_trials
```

Follow each ready/reset/run prompt and do not touch the ball externally during rolling. Both conditions keep the centre attachment (`update_ball_attachment: false`). `baseline` estimates progress from initial ball X plus half the palm travel; `eit` uses sensed contact position to correct virtual progress during manipulation. This tests progress feedback, not moving versus fixed attachment.

### 7. Ball brief-push block: 10 trials

Keep the same controller/configuration running. Establish a repeatable push protocol before this block: use a marked displacement stop or other repeatable jig along the rolling direction, with a consistent externally recorded timing cue during the rolling phase. Choose and record the displacement and timing before collecting results; use them for both conditions. One brief push occurs per trial. Release afterward, preserving palm/ball contact. The controller receives no disturbance flag and does not recover from contact loss.

In terminal C:

```bash
ros2 run franka_eit_vmc run_block.py --task ball --block push --repeats 5 --output ~/eit_trials
```

Ball total: **2 physical conditions × 2 feedback conditions × 5 repeats = 20 trials**. Measure camera-derived final goal error, peak disturbance deviation, recovery into a prespecified goal tolerance, and contact loss. Evaluate recovery within the common 8 s engagement horizon. A baseline can work well during ideal rolling; EIT is expected to help when a push/slip breaks the half-palm-travel assumption.

### 8. Save results and finish

The five blocks produce **50 planned trials**. CSVs are saved under `~/eit_trials/<task>/<block_timestamp>/repeat_XX/`; `block.json` records the block/config check and `outcomes.jsonl` records observation status. Joystick blocks save the continuous ESP32 angle CSV and write `pending_esp32_review` after each completed trial. In automatic mode, plotting waits until the block has ended and the controller stop has been requested, so plotting cannot interrupt the next trial. The runner then saves `joystick_<condition>_<timestamp>_analysis.png` beside each trace, `repeat_XX/comparison.png` for paired repeats, and `block_comparison.png` in the block folder; it also plots a partial attempt after a fault. These plots show the virtual angle, measured point and reference, requested force, raw ESP32 ADC change, and the EIT signal/contact decision sampled by the controller. A red line marks the fault time. The EIT panels show signal versus cutoff, contact and freshness flags, and pattern distance versus model support limit; the fault message also prints these values. A positive signal can still produce no contact if its pattern is outside model support, its predicted point is outside the pad, or the contact hysteresis has not engaged. Older CSVs lack these EIT diagnostic fields, so replotting them can mark the fault but cannot recover the missing EIT evidence. Raw ESP32 counts are not calibrated angles, and pending status does not claim object success. The plots can be regenerated with `ros2 run franka_eit_vmc plot_joystick_results.py <block-folder>`. Ball blocks still ask for a camera-based observation. Record unsuccessful attempts and faults; if rerunning a failed block, keep the original attempt files and identify the additional trials explicitly.

Controller `done` means its sequence finished, not object success. Controller CSVs do not contain measured joystick angle or camera ball ground truth. Define the success tolerance before examining results. Plot joystick errors by offset/condition and ball final errors/recovery traces by disturbance/feedback condition.

To end a running task through the operator tool (terminal C):

```bash
ros2 run franka_eit_vmc experiment.py --task ball --action stop
```

Use `--task joystick` for that controller. The stop ends the FCI task; start a new controller process before further trials. Stop inference with Ctrl+C after robot control ends. A fault also requires inspecting the robot and restarting the controller; no automatic recovery is attempted.

For a single additional trial, with the matching controller launched, use:

```bash
ros2 run franka_eit_vmc experiment.py --task joystick --action ready
ros2 run franka_eit_vmc experiment.py --task joystick --action trial --condition eit --output ~/eit_trials/additional
```

The standalone `move_ready` executable is also available through `task:=ready`; the block procedure already requests ready movement from the task controller, so it does not need a separate FCI owner.

## Git transfer

The root `.gitignore` includes this package, existing physical EIT tools and their two numbered-plate view assets. It excludes paper/simulation campaigns, raw sessions, bags, plots and build products. The runtime needs the `eit_position_toolkit/contact_position_runs` directory on the control computer, or set `EIT_POSITION_RUNS_DIR` to its location. Commit/push from your own repository, then clone/pull on the control computer and rebuild there; never copy a build/install directory between machines.

When transferring to another computer, copy the trained session model files into `eit_position_toolkit/contact_position_runs` there, or point `EIT_POSITION_RUNS_DIR` at their directory. Sensor wiring metadata is embedded in each checkpoint; keep the files unchanged when copying them.

## Offline checks

```bash
cmake -S franka_eit_vmc -B /tmp/eit-core -DFRANKA_EIT_CORE_ONLY=ON
cmake --build /tmp/eit-core
ctest --test-dir /tmp/eit-core --output-on-failure
PYTHONPATH=franka_eit_vmc/python python3 franka_eit_vmc/test/test_eit.py
```

These checks exercise geometry, state updates, condition isolation, stop guards, NPZ portability and number-pair transforms without a sensor or robot. They do not validate gains, mount geometry, localization accuracy, contact dynamics, collision-free reachability or physical closed-loop performance.
