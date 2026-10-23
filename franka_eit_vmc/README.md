# Franka EIT–VMC proof-of-concept package

Two ROS 2 executables implement the matched joystick and ball-rolling experiments:

- `joystick_controller`: centre attachment versus EIT-defined attachment at contact.
- `ball_roll_controller`: robot-motion versus EIT position feedback, with centre attachment shared.
- `move_ready`: move the active pad face above the task at positive Base X/Z and Base Y=0.

The package is self-contained with respect to research code: it contains the controller mathematics, Amodo ARIA serial reader, position model, collector, trainer, inference node, example model, and configuration. Runtime dependencies are ROS 2, Eigen, a compatible libfranka installation, NumPy, PyYAML and pyserial. There are no imports, symlinks or absolute paths back to the original project.

**Model status:** no trained physical-pad model was found in the supplied workspace. `models/demo_position_model.npz` is a trained synthetic UI-signal example, for offline testing only. Robot trials reject demo models. Export your existing measured `position_model.npz`, or collect/train it using the tools below, and put the measured model in `models/position_model.npz`. That directory is intentionally tracked by Git.

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

Copy the installed example configs to an editable location:

```bash
mkdir -p ~/eit_config
cp -r "$(ros2 pkg prefix --share franka_eit_vmc)/config/." ~/eit_config/
```

Set the actual robot IP and **measured** EE-to-active-face translation/rotation in `controller.yaml`. The 140 mm example is not a measured mount dimension. Set `tool_transform_confirmed: true` only after entering that measured transform. Measure tool mass/TCP and configure the Franka robot appropriately using your established procedure. The package does not guess payload or change collision thresholds.

The example workspace is positive Base X/Z, centred at Base Y=0. Ready face centre is `[0.45, 0.0, 0.35]`. Adjust its height to be a small measured clearance above the actual object so the 20 mm approach budget can reach it. `landing_offset_y_m` offsets the joystick landing around the task centre for paired tests.

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

If a physical model has already been trained using the existing contact-position GUI, copy its `position_model.npz` into this package's `models/`. The format is compatible. Inspect it with:

```bash
ros2 run franka_eit_vmc inspect_model.py /path/to/models/position_model.npz
```

To collect a model using only this package, set the actual electrode-to-device pin mapping in `eit.json` (never guess pin order), then:

```bash
ros2 run franka_eit_vmc collect_eit.py --config ~/eit_config/eit.json \
  --output ~/eit_sessions --numbering ccw --points 20 --repeats 3
ros2 run franka_eit_vmc train_eit.py --session /path/to/the/session \
  --output /path/to/franka_eit_vmc/models/position_model.npz
```

The collector prompts for known touch positions and separate unloaded baselines. Use the intended contacting objects/load range and independently verify held-out positions/sessions. The built-in last-repetition holdout is not an unseen-location or cross-session accuracy guarantee. The output uses numeric NPZ arrays and JSON metadata; it does not unpickle arbitrary Python objects.

Start live inference:

```bash
ros2 run franka_eit_vmc eit_contact_node.py --model /path/to/models/position_model.npz \
  --config ~/eit_config/eit.json
```

With the pad unloaded, request 20 fresh baseline frames:

```bash
ros2 service call /eit_contact/calibrate_baseline std_srvs/srv/Trigger '{}'
```

The node publishes `/eit/contact`. Three above-threshold frames indicate contact; two below the lower threshold indicate loss. Novel/out-of-support positions have `position_valid=false`. Clipped/malformed frames are invalid. The controller rejects nonphysical models, invalid frames, wrong pad frames/engraving conventions, stale timestamps, and invalid positions when required by the selected condition. Optionally pin `expected_model_sha256` in the controller config.

The firmware provides no device acquisition timestamp. Published timestamps are host receipt times; software freshness checks do not measure the full electrical acquisition/filter latency. Both processes should use the same host/system clock. The node stops on excessive serial backlog. Actual frame rate must be measured on the device; no 20 Hz hardware rate is claimed.

## Move to ready, then run one experiment

Parsing/dry-run mode is the default and never connects to the robot:

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=joystick config:=/path/to/controller.yaml
```

Once geometry, mounting and posture are configured and other FCI clients stopped, a finite ready move is:

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=ready connect_robot:=true \
  config:=/path/to/controller.yaml mount:=/path/to/mount.yaml posture:=/path/to/posture.yaml
```

Then start **one** experiment node (it initially holds):

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=joystick connect_robot:=true \
  config:=/path/to/controller.yaml mount:=/path/to/mount.yaml posture:=/path/to/posture.yaml
```

In another terminal, with physical inference already calibrated:

```bash
ros2 run franka_eit_vmc experiment.py --task joystick --action ready
ros2 run franka_eit_vmc experiment.py --task joystick --action trial --condition baseline --output ~/eit_trials
ros2 run franka_eit_vmc experiment.py --task joystick --action ready
ros2 run franka_eit_vmc experiment.py --task joystick --action trial --condition eit --output ~/eit_trials
```

`baseline` means centre attachment for the joystick. Repeat at fixed landing offsets with unchanged gains and supplied geometry. `experiment.py` supplies a heartbeat while moving/running, records every attempt to a new CSV, and requests stop on interruption/error. Normal completion leaves the robot holding for the next command. Geometry/gains are immutable while a process is running; restart to apply a config change, and use the condition service/operator tool for the matched baseline/EIT switch.

For the ball, stop the joystick process, launch with `task:=ball`, and use `--task ball` in the operator tool. `baseline` now means robot-motion progress. Set the independently measured `initial_ball_x` in `config/ball.yaml`, or use `overrides:=/path/to/ball_trial.yaml` on launch. Both conditions use the same fixed initial position and goal. Apply the repeatable external push independently; the controller receives no disturbance timing flag.

For evaluation, use independent joystick-angle/ball-position measurements. The CSV records robot/controller state and the robot's external-force estimate, not ground-truth object success or calibrated pad contact force. Optional synchronized recording:

```bash
ros2 bag record /eit/contact /eit_joystick/status /eit_joystick/joint_states
```

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
