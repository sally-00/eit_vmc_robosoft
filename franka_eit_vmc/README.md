# Franka EIT–VMC proof-of-concept package

Two ROS 2 executables implement the matched joystick and ball-rolling experiments:

- `joystick_controller`: centre attachment versus EIT-defined attachment at contact.
- `ball_roll_controller`: robot-motion versus EIT position feedback, with centre attachment shared.
- `move_ready`: move the active pad face above the task at positive Base X/Z and Base Y=0.

The package is self-contained with respect to research code: it contains the controller mathematics, Amodo ARIA serial reader, read-only position decoder, inference node, example model, and configuration. Runtime dependencies are ROS 2, Eigen, a compatible libfranka installation, NumPy, PyYAML and pyserial. There are no imports, symlinks or absolute paths back to the original project.

**Model status:** `models/demo_position_model.npz` is a synthetic UI-signal example for offline testing only. Robot trials reject demo models. Train and export your measured model outside this ROS package, then copy the deployment file to `models/position_model.npz`. That directory is intentionally tracked by Git. The ROS package only loads model parameters and runs predictions; it contains no data collector, trainer or synthetic-model generator.

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

Franka's torque interface already compensates gravity; the package does not add gravity again. A damped null-space projector retains the captured/saved posture. Command targets have configured torque limits and are slewed from the previous hardware command at the configured Nm/s rate. Workspace, point/joint speed, joint margin, external-wrench estimate, tracking error, update-period, operator heartbeat and EIT freshness guards stop the trial. The software workspace bounds the EE, pad centre, selected point and reference; it is not full-arm collision checking.

### Joystick

At electrical contact, the condition chooses the pad centre or the inferred EIT location. This attachment remains fixed in pad coordinates throughout manipulation. The commanded angle `theta_d` drives both the geometric arc and the desired palm orientation:

```
h(theta_d) = h0 + [L*sin(theta_d), 0, L*(cos(theta_d)-1)]
R_pad_desired = rotation_about_Base_Y(theta_d) * R_pad_ready
```

The angle reference has bounded speed and acceleration. It requests +12 degrees, then -12 degrees. `L` is the supplied pivot-to-contact radius (example 92 mm), shared by both conditions. Actual joystick angle is **not** supplied to the controller. Measure it independently for evaluation.

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

`start_trial` approaches downward, engages on electrical contact, ramps preload, and runs the task. At the common time horizon the robot retracts upward and holds. `done` means the commanded trial finished; it does not mean the object met externally measured task-success criteria. Contact loss or sensing/control faults terminate the trial instead of attempting an unvalidated regrasp.

Stop/fault handling slews applied torque toward zero before ending FCI when the callback remains operational. Franka native reflexes and communication failures can terminate control independently. A fault requires restarting the executable; there is no automatic error recovery.

## Build on the robot computer

Put this package anywhere inside a ROS 2 workspace `src/` directory, including inside a clone of this repository. The rest of the paper/simulation directories are unnecessary.

```bash
source /opt/ros/humble/setup.bash  # or your compatible ROS 2 distribution
cd ~/eit_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select franka_eit_vmc --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The direct-FCI interface avoids coupling to a particular `franka_semantic_components` API. Use a realtime-capable host and compatible FCI configuration as for the existing VMC Transformer hardware experiments.

When updating an older installation of this package, build in a clean workspace so removed training executables are not left in the install directory.

Copy the installed example configs to an editable location:

```bash
mkdir -p ~/eit_config
cp -r "$(ros2 pkg prefix --share franka_eit_vmc)/config/." ~/eit_config/
```

Set the actual robot IP and **measured** EE-to-active-face translation/rotation in `controller.yaml`. The 140 mm example is not a measured mount dimension. Set `tool_transform_confirmed: true` only after entering that measured transform. Measure tool mass/TCP and configure the Franka robot appropriately using your established procedure. The package does not guess payload or change collision thresholds.

The example workspace is positive Base X/Z, centred at Base Y=0. Ready face centre is `[0.45, 0.0, 0.35]`. Adjust its height to be a small measured clearance above the actual object so the 20 mm approach budget can reach it. `landing_offset_x_m` offsets joystick landings along Base X for paired tests; keep `landing_offset_y_m: 0.0` so the task stays centred at Y=0.

FR3 joint limits are defaults. For a Panda, supply `config/panda.yaml` as an overlay and verify the actual robot model/limits.

## Engraved numbers and pad orientation

The midpoint between engraved numbers **4 and 5 faces Base +X**. The default configuration uses this mounting direction: `ready_pad_yaw_rad: 1.1780972450961724` (67.5°) and `mount_orientation_confirmed: true`.

For the existing CCW plate, viewed from the contacting face: 1 is bottom, 3 right, 5 top, 7 left. The shipped `config/mount.yaml` records the 4–5 direction. To regenerate that overlay:

```bash
ros2 run franka_eit_vmc configure_mount.py --pair 4 5 --numbering ccw \
  --view contacting-face --output ~/eit_config/mount.yaml
```

The helper computes a downward-facing pad rotation whose selected midpoint points Base +X. It does not measure `ee_R_pad_xyzw`. That extrinsic rotation is the physical relationship between the pad frame and libfranka EE. Avoid applying the same mounting rotation in both quantities.

## Retain the last-joint posture

The reference VMC Transformer controller captures current `q`; it does not supply a fixed numerical q7. To preserve the same posture, capture it while the previous controller/driver is at the desired familiar posture:

```bash
ros2 run franka_eit_vmc capture_posture.py --topic /joint_states --prefix fr3_ \
  --output ~/eit_config/posture.yaml
```

Stop that FCI driver before running this package. Pass `posture.yaml` to the new controller. Without a saved posture overlay (`use_saved_posture: false`), activation captures the current seven joints. Null-space control attracts the robot toward that posture, and a q7 deviation limit (example ±0.35 rad) stops incompatible motions. The package cannot guarantee an arbitrary Cartesian pose is reachable at an exactly fixed q7; it stops rather than forcing a different wrist configuration.

## Physical EIT model and inference

The portable model is an RBF kernel-ridge position decoder on normalized complex-voltage changes. It uses 40 phase/magnitude channels (80 real features), estimates pad XY in millimetres internally, and publishes metres. Force labels/force predictions are not used.

Prepare the model on the EIT training computer in the separate `eit_position_toolkit/` folder, outside this ROS package. For an existing 40-channel RBF checkpoint, the separate `export_franka_model.py` tool in that toolkit writes a deployment NPZ containing precomputed weights, kernel width and support limit:

```bash
python3 /path/to/eit_position_toolkit/export_franka_model.py \
  --input /path/to/trained/position_model.npz \
  --output /path/to/export/position_model.npz
```

Copy that exported file to this package's `models/position_model.npz` on the robot computer, or supply its absolute path. Existing checkpoints without precomputed parameters must be exported first. The current decoder supports RBF models with all 40 adjacent-scan channels; MLP models and other scans are not supported. The loader reads numeric NPZ arrays and JSON metadata with `allow_pickle=False`, and performs no fitting or weight reconstruction.

Inspect the model:

```bash
ros2 run franka_eit_vmc inspect_model.py /path/to/models/position_model.npz
```

Set the actual electrode-to-device pin mapping in `eit.json` to match the exported model. Sensor settings are checked against the model metadata.

Start live inference:

```bash
ros2 run franka_eit_vmc eit_contact_node.py --model /path/to/models/position_model.npz \
  --config ~/eit_config/eit.json
```

With the pad unloaded, request 20 fresh baseline frames. This measures the live electrical reference for prediction; it does not train or modify the model:

```bash
ros2 service call /eit_contact/calibrate_baseline std_srvs/srv/Trigger '{}'
```

The node publishes `/eit/contact`. Three above-threshold frames indicate contact; two below the lower threshold indicate loss. Novel/out-of-support positions have `position_valid=false`. Clipped/malformed frames are invalid. The controller rejects nonphysical models, invalid frames, wrong pad frames/engraving conventions, stale timestamps, and invalid positions when required by the selected condition. Optionally pin `expected_model_sha256` in the controller config.

The firmware provides no device acquisition timestamp. Published timestamps are host receipt times; software freshness checks do not measure the full electrical acquisition/filter latency. Both processes should use the same host/system clock. The node stops on excessive serial backlog. Actual frame rate must be measured on the device; no 20 Hz hardware rate is claimed.

## Run the experiments in order

These commands operate the physical robot. Launching a task connects through FCI and holds the current pose; a ready/trial request starts motion. `task:=ready` starts its finite ready move immediately. All robot examples below use real control. Keep one FCI owner, and source ROS plus `~/eit_ws/install/setup.bash` in every terminal.

### 1. Prepare the hardware and files once

Build using the instructions above and copy the configs to `~/eit_config/`. Set the real robot IP, measured EE-to-pad transform/payload, reachable task height, workspace and joystick length. Set `tool_transform_confirmed: true` after entering the measured transform. The 4–5 midpoint mounting direction is already configured toward Base +X. Capture the desired wrist posture using `capture_posture.py` as described above, before stopping the previous FCI driver; save `~/eit_config/posture.yaml`.

Prepare a physical EIT model in `eit_position_toolkit/`, check its signal diagnostics and test known positions. For this ROS decoder use **eight verified electrode pins, adjacent scan, RBF, all 40 channels usable**; the toolkit's default 16-electrode combined scan/MLP cannot be loaded here. Export with the toolkit's `export_franka_model.py`, copy the deployment file to `~/eit_config/position_model.npz`, and match `~/eit_config/eit.json` to the saved wiring/frequency. Do not guess which eight pins to use on the physical pad.

The ±10 mm X offsets below are example pad-centre displacements. Confirm all three landings stay inside the calibrated sensing region with the real joystick. Base Y stays zero. The block helper checks that the launched offset matches its label. Keep the joystick fixture fixed while changing the palm landing.

### 2. Start EIT inference (terminal A; leave running)

```bash
ros2 run franka_eit_vmc inspect_model.py ~/eit_config/position_model.npz
ros2 run franka_eit_vmc eit_contact_node.py --model ~/eit_config/position_model.npz \
  --config ~/eit_config/eit.json
```

With the pad unloaded, use terminal C to request a baseline:

```bash
ros2 service call /eit_contact/calibrate_baseline std_srvs/srv/Trigger '{}'
ros2 topic echo /eit/contact --once
```

The service queues calibration; inspect a subsequent message after the 20 fresh frames complete. It must show `measurement_valid: true`, `model_is_physical: true`, and `contact: false` while unloaded. When making a known touch, verify contact and position become valid, then release. Before robot trials check that acquisition gaps stay below `sensor_timeout` (default 0.30 s). Recalibrate only while unloaded if the electrical reference drifts.

### 3. Joystick centre block: 10 trials

Set independent joystick-angle recording before the block. In terminal B:

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=joystick \
  config:=$HOME/eit_config/controller.yaml posture:=$HOME/eit_config/posture.yaml \
  overrides:=$HOME/eit_config/joystick_center.yaml
```

In terminal C:

```bash
ros2 run franka_eit_vmc run_block.py --task joystick --block center --repeats 5 --output ~/eit_trials
```

For each trial the helper pauses: clear the local route and type `ready` to move to the ready pose; reset the joystick upright, leave the pad unloaded and start independent recording, then type `run` to approach and execute +12°, then −12°. Enter the independently observed outcome/recording ID afterward. One contact location is used throughout each run. The EIT attachment is defined at contact and stays fixed in pad coordinates. The baseline assumes the pad centre. Both get the same contact detector, gains, length and angle commands. Verify the contact stays approximately fixed on the pad.

Five paired repeats run both conditions, alternating baseline/EIT order across repeats. The helper does not automatically reset objects. `q` at a prompt exits the block while the task controller continues holding; Ctrl+C during a child operation requests its bounded stop and ends the block.

### 4. Joystick negative-X block: 10 trials

After the centre block, stop the joystick controller in terminal B and restart it with the negative-X overlay:

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=joystick \
  config:=$HOME/eit_config/controller.yaml posture:=$HOME/eit_config/posture.yaml \
  overrides:=$HOME/eit_config/joystick_x_minus.yaml
```

Then in terminal C:

```bash
ros2 run franka_eit_vmc run_block.py --task joystick --block x_minus --repeats 5 --output ~/eit_trials
```

### 5. Joystick positive-X block: 10 trials

Stop the controller in terminal B and restart:

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=joystick \
  config:=$HOME/eit_config/controller.yaml posture:=$HOME/eit_config/posture.yaml \
  overrides:=$HOME/eit_config/joystick_x_plus.yaml
```

Then in terminal C:

```bash
ros2 run franka_eit_vmc run_block.py --task joystick --block x_plus --repeats 5 --output ~/eit_trials
```

Joystick total: **3 landings × 2 conditions × 5 repeats = 30 trials**. Measure actual angle error over the common command sequence, final angle error, contact losses and completion; measure peak load only with a reliable independent measurement or clearly label the robot wrench estimate. Expect the fixed-centre baseline to degrade more at off-centre landings; centred performance may be similar. These are hypotheses, not established hardware results.

### 6. Ball preparation and undisturbed block: 10 trials

Stop the joystick controller. Install the ball/table setup, mark its starting position and measure its Base-X centre independently. Set `initial_ball_x` in a copy of `ball.yaml` at `~/eit_config/ball.yaml`; this same value is used for both conditions. Reset the ball to that mark before every trial. The fixed goal is this initial X plus `roll_distance` (default 20 mm). Check the ready height for this object and leave both landing offsets zero in `controller.yaml`.

Start independent camera recording of ball position; it is for evaluation, not controller feedback. In terminal B:

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=ball \
  config:=$HOME/eit_config/controller.yaml posture:=$HOME/eit_config/posture.yaml \
  overrides:=$HOME/eit_config/ball.yaml
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

The five blocks produce **50 planned trials**. CSVs are saved under `~/eit_trials/<task>/<block_timestamp>/repeat_XX/`; `block.json` records the block/config check and `outcomes.jsonl` records your observations. Keep camera/angle recordings with their entered IDs. Record unsuccessful attempts and faults; if rerunning a failed block, keep the original attempt files and identify the additional trials explicitly.

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

The root `.gitignore` includes this package, existing physical EIT tools and their two numbered-plate view assets. It excludes paper/simulation campaigns, raw sessions, bags, plots and build products. `models/*.npz` remains tracked, including a physical model exported later. Commit/push from your own repository, then clone/pull on the control computer and rebuild there; never copy a build/install directory between machines.

After exporting a measured model, explicitly check it is staged with `git status` / `git check-ignore` before transferring. Sensor wiring metadata is embedded in the model. Do not edit `demo=true` to make a synthetic model appear physical.

## Offline checks

```bash
cmake -S franka_eit_vmc -B /tmp/eit-core -DFRANKA_EIT_CORE_ONLY=ON
cmake --build /tmp/eit-core
ctest --test-dir /tmp/eit-core --output-on-failure
PYTHONPATH=franka_eit_vmc/python python3 franka_eit_vmc/test/test_eit.py
```

These checks exercise geometry, state updates, condition isolation, stop guards, NPZ portability and number-pair transforms without a sensor or robot. They do not validate gains, mount geometry, localization accuracy, contact dynamics, collision-free reachability or physical closed-loop performance.
