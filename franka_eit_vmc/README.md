# Franka EIT–VMC proof-of-concept package

ros2 run franka_eit_vmc eit_contact_node.py
ros2 run franka_eit_vmc eit_pad_viewer.py
ros2 launch franka_eit_vmc controller.launch.py task:=joystick   config:=$EIT_CONFIG_DIR/controller.yaml   overrides:=$EIT_CONFIG_DIR/joystick_x_minus.yaml
ros2 run franka_eit_vmc run_block.py --task joystick --block x_minus   --mechanism inward_arc --inward-arc-m 0.002 --axis x   --repeats 1 --output ~/eit_trials
python3 contact_position_app.py --points 30 --repeats 1 --pins 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 --port /dev/ttyACM0  --drive-pattern adjacent


Two ROS 2 executables implement the matched joystick and ball-translation experiments:

- `joystick_controller`: centre attachment versus EIT-defined attachment at contact.
- `ball_roll_controller`: robot-motion versus EIT position feedback, with centre attachment shared.
- `move_ready`: move the active pad face above the task at positive Base X/Z and Base Y=0.

The package is self-contained with respect to research code: it contains the controller mathematics, Amodo ARIA serial reader, position decoder, inference node, measured model, and configuration. Runtime dependencies are ROS 2, Eigen, a compatible libfranka installation, NumPy, PyYAML and pyserial. There are no imports, symlinks or absolute paths back to the original project.

**Model selection:** The EIT node and pad viewer select the most recently modified `position_model.npz` or `position_model_mlp.npz` under `eit_position_toolkit/contact_position_runs/*/` when they start. The selected file path and SHA-256 are printed. The ROS decoder currently supports trained RBF checkpoints; if the newest file is an unsupported MLP checkpoint, loading stops with an error. Use `--model PATH` to select a specific checkpoint. The ROS package does not collect touches or train a model.

The new inward-arc implementation was validated offline. See `VALIDATION.md` for the checks actually performed.

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

Franka's torque interface compensates gravity for the robot and its configured payload. The current workstation has no tool/load mass configured on the robot. `tool_weight_compensation_mass_kg: 0.5` in `controller.yaml` adds **4.903325 N upward in Base Z at the end effector** to compensate for the EIT tool. It is mapped with the EE translational Jacobian, separately from contact-point and alignment forces, in Hold, Ready, Approach, Preload, Interact, Retract and Done, for baseline and EIT conditions. Fault and Stopping retain the existing bounded torque ramp toward zero. Set this parameter to `0.0` if the same weight is subsequently included in Franka's payload model. This constant force does not configure tool inertia or its centre of mass. Restart the controller after changing the mass.

A damped null-space projector retains the captured/saved posture. Joint torque requests go directly to libfranka, which applies its 100 Hz filter and native torque-rate limiter. The package adds no joint-torque magnitude or slew cap. On stop/fault it requests zero torque and waits for Franka's applied desired torque to reach zero before finishing. Workspace, point/joint speed, joint margin, update-period, operator heartbeat and EIT freshness guards stop the trial. The software workspace bounds the EE, pad centre, selected point and reference; it is not full-arm collision checking. Native Franka collision/reflex monitoring remains in force.

The update-period guard allows up to `max_period: 0.010` s during motion and `idle_max_period: 0.020` s while holding or done. The larger idle allowance avoids aborting a completed trial for one tolerated callback gap while leaving the active-motion guard unchanged. A period fault log prints the observed callback interval, limit and mode; libfranka may also stop independently for communication problems.

### Joystick

After confirmed joystick contact, a fresh EIT no-contact decision does not stop or fail the trial, regardless of its duration or a geometric mismatch during that dropout. The task spring and angle trajectory continue. EIT holds the last valid filtered attachment in pad coordinates and resumes filtering when valid contact returns; baseline stays at the pad centre. The separate shaft-snapback release detector still ends an attempt when no-contact coincides with a sufficiently rapid return toward upright while a tilted angle is commanded. Fresh-contact palm-runaway detection and actual sensor/robot faults also retain their response. The previous no-contact grace and mismatch parameters were removed; they no longer determine failure.

Joystick comparison plots retain attempts that fail during Preload or Approach, including `joystick_slip`. Traces align to confirmed contact; attempts that never establish contact align to approach start and are identified as such by the axis label. Failure labels and vertical event markers show when an attempt stopped, and its curve ends there rather than being extended as a successful trial. The comparison time range follows saved data, including the 19-second return-to-zero sequence. Available controller curves remain visible if the ESP32 recording is missing. The block comparison and run summary also include an EIT contact-state subfigure: separate condition lanes show the recorded classifier decision (0=no contact, 1=contact); stale or invalid electrical frames are gaps, and absent contact telemetry is labeled rather than assumed false. This is the detector output, not independent confirmation of physical contact.


At the joystick approach descent limit, `joystick_approach_final_hold_s: 2.0` (in `config/joystick.yaml`) holds the final reference for up to two seconds, with zero reference velocity and no additional commanded descent. Late contact can engage at this held position after the unchanged continuous `approach_contact_confirm_s` period and at least two fresh frames; EIT trials still require a valid in-pad position. A rejected contact resets confirmation. Failure to confirm before this hold ends reports `approach_timeout`; the overall `approach_timeout` deadline also remains active. Setting the hold duration to zero restores immediate distance-limit failure when contact is absent. The ball approach behaviour is unchanged.

The default joystick trajectory now returns to zero before retracting: the target is +20° until 4.5 s after confirmed contact, −20° from 4.5–13 s, then 0° from 13–19 s. These are target changes; the existing velocity/acceleration-filtered angle reference approaches them smoothly. `joystick_return_zero_start_s: 13.0` sets the final phase start and `joystick_duration: 19.0` sets the contact-relative retract time, both in `config/joystick.yaml`. Preload is included in these times. Increasing the duration gives more time to settle toward zero. Setting the return start to `0.0` disables the final phase. The same sequence applies to baseline/EIT/kinematic conditions, either selected tilt axis and both joystick mechanisms. A slip or fault can still terminate a trial early; finishing the desired reference does not establish that the physical shaft returned to zero.

`joystick_update_eit_attachment: true` follows valid EIT contact in both pad X/Y axes during Preload/Interact. `joystick_attachment_filter_tau_s: 0.080` sets each of two cascaded low-pass stages. The filter runs at servo rate with exact held-input updates, initialized at confirmed contact. Larger values are smoother but add lag (80 ms per stage gives roughly 160 ms steady-ramp lag and 310 ms to reach 90% of a step). Invalid, out-of-pad or contact-false readings freeze both stages; valid contact resumes smoothly. The target/arc origin stays fixed at engagement, so updating the attachment changes spring stretch. The point Jacobian and robot material-point damping use the filtered attachment; sensor-position derivatives are not added to damping. This changes the force application point as well as the spring stretch; moving the attachment is not a guarantee of passivity or of exact shaft-angle tracking. Baseline stays at the pad centre and the optional kinematic condition keeps its captured point. Set the switch to `false` to reproduce the old fixed-EIT-attachment comparison. The geometric slip detector continues to use the initial material point so contact migration cannot hide palm runaway.

The default filter was replayed on six recent joystick EIT traces (2026-10-27 22:58–23:12, approximately 50 Hz). Above-5-Hz variation fell from 1.73–3.16 mm RMS to 0.062–0.116 mm RMS. This measures smoothing, not contact-estimation accuracy; genuine fast contact motion is also attenuated. No integral term is added.

The default `joystick_mechanism: legacy` preserves the existing control. At electrical contact, the condition chooses the pad centre or the inferred EIT location. The baseline attachment stays at the pad centre. With `joystick_update_eit_attachment: true`, the EIT attachment follows the filtered current contact during Preload/Interact. The commanded angle `theta_d` drives both the geometric arc and the desired palm orientation:

```
h(theta_d) = h0 + [L*sin(theta_d), 0, L*(cos(theta_d)-1)]
R_pad_desired = rotation_about_Base_Y(theta_d) * R_pad_ready
```

Set the commanded amplitude in `config/joystick.yaml` with `joystick_angle_deg` (currently 20.0); the controller converts it to radians internally and requests +angle, then -angle. The value must be finite and positive; there is no additional command-angle cap in this package. The angle reference has bounded speed and acceleration. `L` is the supplied pivot-to-contact radius (`joystick_length`), shared by both conditions. In legacy mode the measured angle is recorded for evaluation and does not drive control. The optional `inward_arc` mode below uses live measured shaft angle for alignment.

Horizontal spring–damper forces act at the selected point. A separate normal spring–damper ramps a nominal downward preload (example 3 N), and orientation control follows `theta_d`. Preload is an impedance bias, not force feedback from EIT. Both conditions receive the same contact detector, trajectory, gains, supplied geometry and limits.

### Choose the joystick tilt / rotation axis

Set `--axis` in the `run_block.py` command:

| Option | Shaft tilt / tip motion | Positive-angle rotation |
| --- | --- | --- |
| `--axis x` | Base +X, then −X (original trajectory) | About Base +Y |
| `--axis y` | Base +Y, then −Y (other trajectory) | About Base −X |

Add `--axis y` to tilt toward Base +Y and then −Y; `--axis x` selects the existing +X/−X motion. Here the axis names describe **shaft tilt/tip displacement**: positive X tilt rotates about Base +Y, and positive Y tilt rotates about Base −X. This option works with both `legacy` and `inward_arc`. If omitted, the controller keeps `joystick_axis` from `config/joystick.yaml` (default `x`) or its last acknowledged selection. The block landing location (`--block x_plus`, etc.) is independent of this choice. No different launch overlay is needed to select the other tilt axis.

```bash
ros2 run franka_eit_vmc run_block.py --task joystick --block x_plus \
  --mechanism inward_arc --inward-arc-m 0.005 --axis y \
  --repeats 1 --output ~/eit_trials
```

Axis changes are accepted only in Hold/Done before motion. The runner waits for the realtime controller to report the requested selection before starting the recorder or ready move. Each block records the axis in `block.json` and trial CSVs; angle, displacement and palm-tilt plots use that selected axis. Existing CSVs without an axis field retain their historical Base-X interpretation. Rebuild and install after changing controller code, then restart the controller before using the new options.

### Optional joystick mechanism: both arcs inward

Use `run_block.py --mechanism inward_arc --inward-arc-m 0.010` to select the new mechanism and shorten **both** virtual radii by 10 mm. Omit the depth argument to use `joystick_inward_arc_m` from `controller.yaml`. Omitting both arguments keeps the controller's current selection; `--mechanism legacy` explicitly restores the original mechanism. These options apply only to joystick blocks. Mechanism/depth updates are accepted only in Hold/Done before motion, and the runner waits for acknowledgment from the realtime controller.

Add `--include-true-position` to include a third contact-position condition alongside baseline and EIT:

```bash
ros2 run franka_eit_vmc run_block.py --task joystick --block y_minus --axis y \
  --mechanism inward_arc --inward-arc-m 0.003 \
  --include-true-position --repeats 1 --output ~/eit_trials
```

The additional condition is labeled **kinematic tip reference** (`kinematic` in filenames and logs). Its Base-frame tip position is `joystick_pivot_m + joystick_length * measured_shaft_direction`. At confirmed contact, the controller transforms this point into the measured palm frame and projects it onto the contacting plane. This chooses the task spring's force application point once; it stays fixed in palm coordinates for the rest of the trial, The EIT condition instead follows filtered contact by default. Baseline still uses the palm centre. Shaft alignment continues to follow both measured angles. The tip position is a geometry-based reference, with accuracy depending on the pivot, length and ADC calibration; it is not an independent external measurement of the physical contact point.

All three conditions use the same EIT contact detection, ready/baseline calibration, controller gains and commanded trajectory. The kinematic condition requires fresh shaft feedback and rejects a projected tip outside the configured active pad radius. Each repeat runs all three conditions; six repeats cycle through all six possible orders. Without the flag, the existing two-condition sequence is preserved. The option also works with the legacy mechanism, which needs fresh shaft feedback for this condition. `contact_source` and `joystick_tip_position_m_x/y/z` are recorded in trial CSVs, and summaries/comparisons include the third condition. Rebuild/install and restart the controller after this interface update.

The physical `joystick_length` is unchanged. With the extended physical radius `L=0.1025` m and inward depth `delta=0.010` m, both virtual radii are 0.0925 m. The depth must satisfy `0 <= delta < L`. This is the same geometric design as the simulation, using the hardware's own dimensions and existing force limits rather than the simulation's 92 mm joystick.

```text
s(theta_x, theta_y) = normalize([tan(theta_x), tan(theta_y), 1])
main task target = h0 - L*[0,0,1] + (L-delta)*s(theta_x_command, theta_y_command)
alignment target = measured_physical_pivot + (L-delta)*s(theta_x_measured, theta_y_measured)
```

Those are the palm-attachment task target and the alignment target. The active `joystick_task_spring_uses_shaft: false` mode uses the displacement from the selected palm attachment to its commanded arc reference for all three spring components. The damper uses commanded arc velocity minus the velocity of that same attachment. A pad that travels past its reference therefore receives a restoring force, even when the joystick shaft remains upright. The optional `true` mode instead uses measured-versus-commanded shaft displacement for the Base X/Y spring components; its Z component still holds the palm near the contact-defined arc.

The main reference is already `delta` inward at the upright pose; it does not merely reduce displacement along an arc starting at the surface. At contact, the controller derives its task pivot from the measured initial X/Y shaft direction, so an initial tilt is preserved instead of being silently set to zero. It then commands the selected Base tilt axis toward the absolute ±`joystick_angle_deg` targets and the other axis toward zero. The task virtual pivot remains contact-defined. The alignment frame follows the **current measured Base X and Base Y shaft angles** and rotates the palm face perpendicular to the resulting 3D shaft direction; it does not use the final goal or commanded angle. Its physical pivot comes from `joystick_pivot_m` and stays fixed across offset blocks. The shaft was extended by 82.5 mm from 20 mm to 102.5 mm. Its pivot remains `[0.45,0,0.035]` m, giving an upright tip at Base Z=0.1375 m. Enter the measured pivot for the actual fixture.

Four spring–damper links join the alignment frame to one common massless planar slider on the palm. All four anchors move together. Tangential force balance sets the slider velocity; the guide transmits normal force and rotation, plus tangential force at its radial stop. The main task force is mapped at the centre/EIT contact attachment, the alignment resultant at the slider centre, and the alignment couple through the angular Jacobian. Both joystick mechanisms can update the EIT attachment using the same filter.

`joystick_alignment_half_span_m: 0.160` places each virtual alignment anchor 160 mm from the common centre (320 mm between opposite anchors), twice the previous 80 mm distance. It applies to both the palm and target cross-arms for either tilt axis. The assembled alignment moment still uses `moment_limit`.

The new mechanism uses one isotropic 3D task spring with `joystick_task_spring_k` (N/m) and one scalar damper `joystick_task_spring_d` (Ns/m). Its elastic force is radially saturated at `joystick_task_spring_force_limit` (N), then its independent damper force is added. Each of the four alignment links independently saturates its own elastic force at `joystick_alignment_spring_force_limit` (N per link), then adds its damper force. The free-slider velocity is solved from these actual saturated elastic forces so tangential balance is preserved. The forces are added without a cap on their sum. The force limits bound the elastic terms only; damping can make a link's total force exceed its elastic limit.

`joystick.yaml` sets `joystick_alignment_enabled: true`, enabling the four-link orientation guide during `inward_arc` Preload/Interact. Set it to `false` and restart the controller for a target-spring-only comparison: this removes the four alignment forces, their orientation moment and slider motion, while the task spring and its damper still act at the selected centre/EIT/kinematic palm attachment. A force applied away from the EE origin still creates joint torque through the contact-point Jacobian. Ready, Approach and Retract retain their orientation controls. This switch has no effect on `legacy` or the ball task.

With a fixed attachment, the palm-attachment spring can reach its reference while the shaft is at a different angle if contact moves across the pad. The default EIT mode now follows that migrating contact with a filter. Fresh shaft feedback remains required and is used to identify contact release and geometric slip; it is not the active spring stretch. The elastic vector force remains capped by `joystick_task_spring_force_limit`, with the separate palm-velocity damper added afterward. There is no integral. The controller can apply tangential force to the joystick only through actual palm contact.

In either spring mode, while EIT reports contact, a suspected palm runaway is detected if the captured material point remains more than `joystick_slip_contact_mismatch_m` (20 mm) ahead of the shaft along the commanded arc displacement for `joystick_slip_confirm_s` (60 ms). A shaft that tilts farther than the palm does not satisfy this condition. This check retracts and records `failed_joystick_slip`. During EIT no-contact reports this geometric failure check is inactive; release instead requires the separate shaft-snapback evidence described above. These are controller consistency checks, not independent camera confirmation.

```text
F_task = sat_norm(k_task * position_error, F_task_max) + d_task * velocity_error
F_alignment_i = sat_norm(k_align * link_extension_i, F_align_max) + d_align * relative_velocity_i
```

Both branches ramp over `preload_time`. `coupling_k/d`, `normal_k/d`, `force_xy_limit`, `normal_force_limit`, `preload` and `orientation_k/d` are legacy contact parameters and do not govern inward-arc contact forces. Hold, Ready, Approach and Retract retain their separate controls. The historical `joystick_force_scale` field is always 1; the two new fields `joystick_task_spring_scale` and `joystick_alignment_min_spring_scale` report the independent elastic saturations.

Set the task and per-link alignment elastic-force limits in `controller.yaml`. The installed configuration uses 15 N for the task spring and 20 N per alignment link; review these alongside the remaining limits below. These values cap elastic force only, with damping added separately.

### Remaining limits for review

The current workstation configuration also applies these limits to the new VMC:

| Parameter or mechanism | Current value | Where applied |
| --- | --- | --- |
| `moment_limit` | 5 Nm | Norm of the assembled alignment moment, including damping; also Hold/Done/Retract orientation |
| `ready_moment_limit` | 5 Nm | Ready orientation moment |
| `approach_moment_limit` | 15 Nm | Approach orientation moment |
| libfranka torque filter | 100 Hz | Default low-pass filter after the package callback |
| libfranka torque-rate limiter | approximately 1000 Nm/s per joint | SDK limiter enabled by this controller |
| Native Franka limits/reflexes | Robot configuration | Collision/reflex, torque and other hardware monitoring |

Hold/Ready/Approach/Retract translation has no package force-magnitude cap, and there is no additional package joint-torque magnitude cap. The separate speed, workspace, joint-margin, heartbeat and sensor-freshness guards also remain active. Native collision threshold values are not recorded by this package.

Parameters specific to this mechanism are grouped in `config/controller.yaml`: inward depth, physical pivot, alignment half-span and per-link gains, and slider travel are marked `NEW ONLY`. Feedback topic and timeout are marked `NEW CONTROL ONLY` because legacy also records their diagnostics without using them for control. Shared parameters and legacy-only contact gains are also identified in comments. Restart to change geometry/gains; only the mechanism selector, inward depth and tilt axis may change between trials.

The block runner starts the existing single-owner ESP32 recorder and a `joystick_feedback_node.py` process that tails its CSV. No second process opens the serial port. The calibrated Base-X and Base-Y tilts define the 3D shaft direction; Base-X tilt corresponds to positive shaft rotation about Base +Y. The same measured angle/rate is available to both EIT and non-EIT conditions. CSV receipt timestamps are preserved through ROS publication, so replayed or backlogged file records do not become fresh by being republished. USB acquisition/transport delay is not independently measured. The angle is piecewise-linear from the supplied ADC calibration, and rate uses a 50 ms first-order filter. Upright ADC overrides supplied to the block runner also apply to feedback conversion.

Stale or invalid feedback on either joystick axis blocks trial start or stops an active new-mechanism trial; ADC-rail readings remain valid bounded ±35° estimates and retain the `clipped` diagnostic flag. Inward-arc trials can start from a stable tilt, including at an ADC rail: the task reference begins at the measured X/Y angles, then commands the selected tilt axis toward +`joystick_angle_deg` and −`joystick_angle_deg` while driving the other axis toward 0°. The palm alignment target follows both measured axes throughout. The current calibration covers approximately ±35° and reaches ADC rails at the endpoints, so run a command **below** that range, such as 30°. `run_block.py` checks this before motion. Rebuild the package before using these options because the status message and feedback interface changed.

For example, after starting EIT inference, launch the centre-block controller in terminal B:

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=joystick \
  config:=$EIT_CONFIG_DIR/controller.yaml \
  overrides:=$EIT_CONFIG_DIR/joystick_center.yaml
```

Then in terminal C:

```bash
ros2 run franka_eit_vmc run_block.py --task joystick --block center \
  --mechanism inward_arc --inward-arc-m 0.010 --axis x --repeats 5 --output ~/eit_trials
```

For off-centre blocks, restart with `joystick_x_minus.yaml`, `joystick_x_plus.yaml`, `joystick_y_minus.yaml` or `joystick_y_plus.yaml` and use the corresponding `--block` name. Keep `joystick_pivot_m` fixed. To compare the old mechanism at the same 30° command, restart the same launch and run the block with `--mechanism legacy`. Automatic blocks still stop the controller when finished, so each new block needs a new launch as in the original procedure.

### Contact release with shaft snapback

In inward-arc mode, an unsuccessful trial retracts early when fresh EIT feedback reports no contact, the measured shaft returns within `joystick_release_upright_deg` (5°) of upright after a drop of at least `joystick_release_drop_deg` (10°) within `joystick_release_window_s` (0.10 s), and the reference still commands at least `joystick_release_min_command_deg` (10°) of tilt. These parameters are in `controller.yaml`. This is an operational release detector, not independent proof of physical contact loss. An electrical dropout alone or a normal commanded return through zero does not trigger it.

The controller captures the current palm orientation and starts the existing bounded upward retraction before generating another shaft-alignment command. After complete retraction, `run_block.py` records `unsuccessful_contact_release` with `success: false` in `outcomes.jsonl` and continues to the next scheduled condition/repeat. CSVs retain `trial_contact_released`; trial plots mark the event and label the attempt unsuccessful. The failed attempt counts as that scheduled trial and is not automatically retried. The next condition still performs its ready move, fresh baseline calibration and stable-shaft checks.

For the shaft-task mode's separate slip detector, the CSV also records `joystick_slipped=True`; the block records `failed_joystick_slip` and continues after retraction. A contact dropout alone does not classify a slip.

A native Franka reflex, stale/invalid feedback, incomplete retraction or another controller fault still stops the block. There is no automatic fault recovery. Restart the controller after rebuilding this interface change.

### Joystick angle recordings

Every joystick block saves `run_summary.png` when it finishes, fails or is interrupted with Ctrl+C. A startup/ready failure still produces a diagnostic summary when no trial or ESP32 samples exist. Partial trials are labeled `INCOMPLETE`; explicit controller faults are labeled `FAULT`. `block_result.json` records the exit status, stage, condition and error. Cleanup errors are reported separately so the original trial error is preserved and plotting is still attempted. Normal OS termination such as a forced kill or power loss cannot run exit cleanup; regenerate plots from the saved files with the command below.

When controller CSV traces exist, block cleanup also renders `vmc_replay/repeat_XX/` MP4s for each joystick condition and a baseline/EIT pair, including partial trials after a fault. `vmc_replay/joystick_vmc_replay.mp4` is the latest available pair or single trial. `vmc_replay/metrics.json` links each replay to its source trace. The animation reconstructs the robot, pad, shaft and virtual springs from saved telemetry; it is not camera footage. The ±35° ESP32 endpoint values are clipped calibration estimates. To regenerate videos for a saved block:

```bash
ros2 run franka_eit_vmc replay_joystick.py ~/eit_trials/joystick/<run-folder> \
  --output ~/eit_trials/joystick/<run-folder>/vmc_replay
```

Add `--show-plots` to any `run_block.py` command to open the saved summary on the desktop after exit, including an error exit. Plots are always saved when the exit handler runs. To regenerate the summary and available trial/comparison plots:

```bash
ros2 run franka_eit_vmc plot_joystick_results.py ~/eit_trials/joystick/<run-folder>
```

Every joystick block now saves both `joystick_esp32.csv` (the original raw ADC recording) and `joystick_calibrated.csv`. The latter retains source timestamps and raw counts and adds `shaft_angle_rad`, `shaft_angle_deg`, `shaft_rate_rad_s`, the corresponding Base-Y angle/rate columns, `measurement_valid` and `clipped`. Calibration comes from the block's copied `joystick_adc_calibration.json`; the calibrated shaft angle describes tip motion in Base X, or shaft rotation about Base Y. Values at an ADC rail/calibrated endpoint retain a bounded ±35° angle estimate and are flagged `clipped`, but do not by themselves invalidate feedback or stop the controller. The first sample has no measured rate and is marked invalid. Keep the validity flags when evaluating errors.

Trial CSVs also record `tool_weight_compensation_force_N_x/y/z`; `requested_force_N_x/y/z` includes that extra force. `coupling_force_N` retains the task-spring contribution separately. These are requested forces before SDK torque filtering/rate limiting.

Both mechanisms record this calibrated stream. Only `inward_arc` uses it for control. Trial CSVs also record the controller's received angle/rate, feedback age/freshness/clipping, actual mechanism/depth, slider position and travel-stop flag, alignment force/target/centre and combined-force scale. These fields distinguish the measured shaft angle from `theta_d_rad`, the commanded angle. The shared clock allows alignment with raw joystick, EIT decision and robot traces. The new mechanism's shaft sensor is feedback, so it is not an independent ground-truth validation of that same sensor.

The recorder is stopped first at block end, then the feedback node drains remaining complete samples and closes the calibrated CSV. The runner starts these processes automatically; do not start another joystick reader or feedback publisher for the same block. Standalone `joystick_hardware/run_joystick.sh` still saves its original raw format.

### Ball translation with sliding

The ball controller now uses the simulation's **direct horizontal goal coupling
and upward-yielding normal carriage**. Both position-feedback conditions share
contact recovery and centre attachment by default; baseline means without EIT
position feedback, not without EIT contact detection.

```
robot observation = independent initial ball coordinate + ball_baseline_motion_ratio * palm travel
EIT observation   = selected Base coordinate of the sensed contact
F_axis = goal_k * (moving_spring_goal - filtered_observation) - ball_goal_d * estimated_velocity
Fn = clip(normal_k*(zn - point_z) + normal_d*(vn - point_velocity_z), -normal_force_limit, 0)
normal_mass * acceleration = -preload - normal_damping*vn - Fn  (normal brake engaged)
normal_mass * acceleration = -preload - normal_damping*vn       (recovering without contact)
```

Both observations update on fresh contacting electrical frames, with position
and velocity filters 0.35/0.25. During loss they hold the last estimate. The goal
is the selected `initial_ball_x/y` plus signed `roll_distance`; `ball_axis: 'x'`
or `'y'` selects direction. The fixed goal does not move when contact updates.
There is no horizontal mass or estimate-correction rebase. The baseline ratio
is `1.0`: the ball is assumed to move with the pad while sliding on the base.
The EIT condition uses the sensed contact position. Both assumptions remain
estimates; the independent camera records actual ball travel. Historical blocks
without this parameter retain their earlier 0.5 rolling ratio in replay.

With `ball_recover_contact=true` (default), **both** estimators pause/damp horizontal
driving after `ball_interact_dropout_grace_s: 0.04` s of continuous no-contact or
invalid EIT position, then release the normal carriage on contact loss. Isolated
bad frames keep the last position estimate and normal brake. Stale or invalid
electrical measurements still fault immediately. The unloaded
carriage descends under its virtual load without treating the commanded spring
force as a contact reaction. A separate 50 N/s pause-force slew unwinds the
lateral drive when contact drops; drive engagement retains its 10 N/s slew.
Contact engages a
one-way brake: descent is blocked, but virtual spring reaction can raise the
carriage. A sustained invalid EIT position with contact still reported pauses
lateral drive but keeps the normal brake. Contact return resumes driving without waiting through a second
settling period. The original initial approach and `preload_time` /
`ball_contact_settle_s` settling gate remain. A bounded approach-force match
initializes the normal spring to avoid unloading at handoff. Invalid/stale
sensing stops; it is never interpreted as permission to descend. Normal travel,
workspace, speed, joint, heartbeat and SDK protections remain.
During the initial preload blend, the ball spring starts at the bounded
approach force and transitions over `preload_time` to the configured 3 N
virtual load and 4 N steady force cap. The transient request cannot exceed
`ball_approach_force_limit`; the 4 N cap applies after that blend.
The recovery reference stops at the higher of its configured travel limit and
the pad workspace floor (with a 0.1 mm margin). If contact has not returned at
that point, the controller reports `contact_loss` rather than commanding a
reference below the workspace floor.

Both ball conditions use the same signed `roll_distance` goal, contact recovery,
and `ball_duration` time limit. Once the filtered controller estimate has
progressed at least the goal travel minus `ball_goal_tolerance_m` for
`ball_goal_dwell_s` with valid contact on
at least two sensor frames, the robot retracts. If the estimate does not reach
the goal, it retracts at the time limit. The controller reports `ball_goal_reached`
in ROS status and the trial CSV. This reports an internal estimate; camera-measured
ball travel remains the independent result.
Both conditions use the same moving spring equilibrium. After stable contact,
the spring goal starts at the initial ball coordinate and moves toward the fixed
stopping goal at 30 mm/s during valid lateral drive. The last paired run moved
the pad about 56–68 mm in its first 2 seconds, which motivated this ramp speed.
Once the reference reaches the stopping goal, it moves up to 50 mm farther at
4 mm/s while the estimated ball is still short. The 10 cm −Y overlay allows
this extension over the full 10 cm remaining error; other configurations can
restrict it with `ball_reference_advance_start_error_m`. It freezes during
rejected sensor frames and contact recovery. The controller still stops only
when the estimated ball position reaches the original goal (within the configured
tolerance and dwell); the moving equilibrium remains subject to the existing
5 N force cap and 10 N/s drive slew. This is a prescribed reference motion,
not accumulated position-error feedback or camera feedback. A moving reference
can supply energy to the task, so it is not a strict passivity guarantee.
The −Y 10 cm overlay uses a common 20 s maximum so a short recovery does not
consume the entire sliding opportunity; the software workspace still bounds the
pad to Base Y ≥ −0.32 m.

Set `ball_recover_contact=false` only for the recovery ablation: after initial
settling it holds the one-way brake and continues horizontal drive during loss.
Compare those recovery policies with EIT position feedback in both. The paired
position-feedback block rejects this override to prevent confounding.
`ball_initial_attachment=true` defines the attachment at initial EIT contact;
`update_ball_attachment=true` additionally follows it. Updates preserve normal
and cross-axis spring extension and the paused horizontal anchor; attachment
velocity is the material point velocity, not contact migration velocity.
The position-feedback block requires both attachment options false.

**Existing parameter files and values were preserved.** The old `virtual_mass`,
`virtual_damping` and `progress_speed` are still accepted for compatibility but
no longer drive a horizontal mass. `ball_contact_loss_grace_s` still applies to
provisional approach dropouts; it no longer terminates bounded rolling recovery.
Existing `goal_k/goal_force_limit` now act directly on horizontal palm force;
`coupling_k/d` retain cross-axis stabilization. Thus the same numerical gain can
have a different effect: the existing 10 N goal cap now permits up to 10 N palm
drive, rather than approximately 5 N through the old transmission. The existing
`force_xy_limit` also bounds it. These commands are not measured contact forces.

Ball control settings, including defaults and the 10 cm −Y overlay:

| New parameter | Default | Role |
|---|---:|---|
| `ball_goal_d` | 20 Ns/m | Damping from estimated ball velocity |
| `ball_force_slew` | 10 N/s | Horizontal force slew while driving |
| `ball_pause_force_slew` | 50 N/s | Horizontal force slew while paused for lost contact |
| `ball_baseline_motion_ratio` | 1.0 | Estimated ball travel per pad travel for sliding baseline |
| `ball_interact_dropout_grace_s` | 0.04 s | Ignore isolated bad frames after engagement |
| `ball_reference_speed_m_s` | 0.030 m/s | Constant spring-goal speed toward the stopping goal |
| `ball_reference_advance_speed_m_s` | 0.004 m/s | Spring-goal advance after the reference reaches the fixed goal |
| `ball_reference_advance_limit_m` | 0.050 m | Maximum extension beyond the stopping goal |
| `ball_reference_advance_start_error_m` | 0.050 m; 0.10 m in the 10 cm −Y overlay | Largest remaining estimated error that permits post-ramp extension |
| `ball_goal_ramp_s` | 0.4 s | Initial drive engagement ramp |
| `ball_normal_mass` | 0.4 kg | Virtual normal inertia |
| `ball_normal_damping` | 20 Ns/m | Grounded carriage damping |
| `ball_normal_speed` | 0.015 m/s | Carriage speed bound |
| `ball_normal_travel` | 0.025 m | Downward carriage travel, additionally bounded by workspace floor |
| `ball_normal_up_travel` | 0.002 m | Upper bound above initial anchor |
| `ball_pause_k`, `ball_pause_d` | 300 N/m, 35 Ns/m | Horizontal contact-loss hold |
| `ball_position_filter`, `ball_velocity_filter` | 0.35, 0.25 | Observation filters |

The existing `preload` sets the downward virtual load; existing `normal_k/d` and
`normal_force_limit` bound its coupling. Upward yielding uses robot pose and
virtual state, not physical force feedback. Hardware startup, physical decoder,
actuation, configured gains/limits and X/Y options differ from the simulation;
matching the VMC structure does not establish identical hardware performance.
New CSV/ROS fields record normal carriage position, velocity, origin, brake,
travel-stop status, normal command and estimated ball velocity. `progress_m`
now reports the filtered ball estimate, not a virtual carriage coordinate.

For a 10 cm goal toward Base −Y from the measured ball start Y=−0.135 m, use `ball_y_minus_10cm.yaml` as the launch overlay. It sets `ball_axis: 'y'`, `initial_ball_y: -0.135`, `roll_distance: -0.10` and a 20 s maximum. The ball goal is Y=−0.235 m; the nominal sliding-baseline pad endpoint is also Y=−0.235 m. `ball.yaml` puts the ready pad over the new start at Y=−0.135 m. The overlay extends the software Y minimum to −0.32 m, allowing recovery margin. Review the whole route before launching; the workspace does not prove reachability or obstacle clearance. The configuration validator checks the ball goal and nominal pad endpoint using the configured motion ratio.

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=ball \
  config:=$EIT_CONFIG_DIR/controller.yaml \
  overrides:=$EIT_CONFIG_DIR/ball_y_minus_10cm.yaml
```

Then run the usual ball block in another terminal:

```bash
ros2 run franka_eit_vmc run_block.py --task ball --block undisturbed \
  --repeats 1 --output ~/eit_trials
```

The active EIT pad is 90 mm across. The 1:1 sliding baseline keeps the estimated ball over the same pad location as it moves. If the pad slips over the ball, that estimate is wrong; the EIT contact position and camera record help reveal the discrepancy. Both conditions pause lateral drive and search for contact when the EIT detector reports valid contact loss. Ball trials record `ball_axis`, both goal coordinates, the motion ratio and the selected scalar goal; the runner prints the nominal pad stroke before motion. Restart the controller after changing the configuration.

Optional attachment updates exist for a later migration experiment. They filter/bound the contact change and rebase the reference and normal anchor. Leave them disabled for the two-condition ball proof of concept.

### Trial states and stopping

Each experiment starts holding its current pad pose. `move_ready` lifts to the ready height, translates/orients above the task, and descends to the configured ready height. This is a bounded local Cartesian route, not a collision-aware motion plan. The starting EE and active pad face must already lie inside the configured workspace. A trial requires the robot to be at that ready pose and the pad to be electrically unloaded.

The positioning spring/damper force during hold, ready movement, approach and retract has no combined Cartesian force cap. Ball approach uses this original tracking law; the 4 N normal cap begins only after confirmed contact. Legacy trial normal force retains `normal_force_limit` and legacy trial horizontal force retains `force_xy_limit`; inward-arc mode uses the independent elastic-spring saturations described above. These are controller command limits, not measured contact-force limits. Joint speed and workspace guards remain active. The controller no longer stops solely because the moving reference is far from the measured pad position; it still requires the pad to be at the ready pose before starting a trial.

Ready movement and retraction use `ready_k/d` for Cartesian position control; Hold and Approach use `hold_k/d`. Ready orientation uses `ready_kr/dr` and `ready_moment_limit`. Approach orientation uses `approach_kr/dr` and `approach_moment_limit`, as does legacy Preload. Legacy Interact uses `orientation_k/d` and `moment_limit`; inward-arc Preload/Interact use their independent task/alignment springs and the assembled alignment `moment_limit`. Hold, Done and Retract orientation use `hold_kr/dr` and `moment_limit`. Ball rolling also uses `orientation_k/d` and `moment_limit` to command the pad toward its ready orientation; pad tilt alone does not stop the trial. Read the active YAML for gain and limit values. The SDK torque-rate limiter bounds changes in the applied joint torque. The ready route advances after the pad settles within 5 mm of each waypoint and meets the orientation/velocity checks; trial start also accepts up to 5 mm.

The package does not apply its own stop threshold to Franka's estimated external force or moment. Status and ready/trial CSVs retain both estimates for inspection. They also record the controller's requested Cartesian force and moment and the final commanded joint torques. The requested Cartesian values precede Jacobian mapping and SDK torque filtering/rate limiting, so they are not measured contact forces. Franka's native collision/reflex behavior remains configured on the robot.

`start_trial` approaches downward. At the first valid electrical contact report, it slows to `approach_contact_speed` and commands another `approach_contact_travel_m` (defaults 5 mm/s and 1 mm). Preload begins after EIT contact remains valid for `approach_contact_confirm_s` (default 0.25 s) across at least two frames and the verification descent completes. The pad may move down or meet resistance; both responses count. If EIT contact disappears, normal approach resumes. The total approach distance and time limits still apply. These signals support contact establishment but do not independently prove physical contact. For joystick trials, an isolated EIT no-contact report after engagement does not stop the controller: baseline keeps the pad-centre attachment, and EIT keeps the established contact position. In inward-arc mode, a combined no-contact report and rapid shaft snapback ends the attempt as described below. The ball EIT condition pauses on brief fresh contact dropouts and stops after a sustained 0.50 s loss. Baseline records those reports but continues after initial contact confirmation. At the common time horizon the robot retracts upward and holds. If retraction times out but the EIT stream has reported no contact continuously for at least 0.5 s, the controller holds its measured pose and reports `done` with `retract_incomplete=true`; the next ball block repeat starts its ready move automatically; `--manual` restores the ready prompt. A stale stream or continued contact still produces `retract_timeout`. `done` means the commanded manipulation finished; it does not mean the object met externally measured task-success criteria. The CSV records whether retraction was incomplete. Check the physical route before the next ready move.

In legacy mode, during preload the horizontal contact-point spring is active immediately while the downward preload ramps. The orientation target remains continuous with approach and the joystick angle is commanded relative to the measured ready orientation. This avoids the orientation-command jump and weak initial lateral support visible in the failed x_plus traces.

Stop/fault handling requests zero torque through the SDK filter/rate limiter and waits for the applied desired torque to reach zero before ending FCI when the callback remains operational. Franka native reflexes and communication failures can terminate control independently. A fault requires restarting the executable; there is no automatic error recovery.

## Build in the current workspace

This checkout is at `~/franka_ros2_ws/src/eit_vmc_robosoft/franka_eit_vmc`. The package can also be placed in another ROS 2 workspace `src/` directory; keep the sibling `joystick_hardware/` directory alongside it when building. The build installs its ESP32 recorder beside the block runner, so the installed program can locate it without the source checkout. The paper/simulation directories are unnecessary.

This computer uses Ubuntu 24.04, ROS 2 Jazzy and system Python 3.12. Deactivate Conda before running ROS tools (`conda deactivate` until `(base)` is gone). Explicitly select `/usr/bin/python3` when building so an old cache cannot select Conda. The same Python must be first on `PATH` for scripts using `#!/usr/bin/env python3`.

The robot at `192.168.8.185` reports FCI protocol **9**. This workspace uses **libfranka 0.17.0**, **franka_ros2 tag v3.0.0**, **franka_description 2.0.0** and **ros2_control 4.48.1**. libfranka 0.18 and later require protocol 10 and cannot connect to this robot. The source workspace's `franka.repos` pins the dependencies. Its bundled older `controller_manager` is excluded with `COLCON_IGNORE`; the manager from ros2_control 4.48.1 supports the configured blocking hardware reads and matches the installed Jazzy controller interfaces.

```bash
cd ~/franka_ros2_ws
bash src/build_franka.sh
source install/setup.bash
```

The helper builds the Franka stack, MoveIt/Gazebo integration and this package, then runs this package's offline tests. It also adds `/opt/ros/jazzy` to CMake's search path so the installed Pinocchio dependency can be found. It never starts a robot controller.

The wired interface `enp6s0` reaches the robot directly on the `192.168.8.0/24` LAN at 1 Gbit/s, full duplex. The current DHCP address is `192.168.8.164`; Wi-Fi supplies the preferred Internet route. No IP or routing change was needed. A read-only check on 2026-10-25 received a 1 kHz robot-state stream, and the rebuilt SDK successfully loaded the robot model and evaluated mass, Coriolis, gravity and the EE Jacobian. No motion was tested. Before physical control, boot the realtime kernel and enable realtime scheduling permissions; this session's `ulimit -r` was 0.

The direct-FCI interface avoids coupling to a particular `franka_semantic_components` API. Use a realtime-capable host and compatible FCI configuration as for the existing VMC Transformer hardware experiments.

When updating an older installation of this package, build in a clean workspace so removed training executables are not left in the install directory.

Edit configuration in the package source directory. Set this path in every terminal used below:

```bash
export EIT_CONFIG_DIR=$HOME/franka_ros2_ws/src/eit_vmc_robosoft/franka_eit_vmc/config
```

On this computer, `.bashrc` exports this path for new terminals. Run `source ~/.bashrc` in terminals that were already open. The controller launch and block runner must use the same configuration directory; otherwise the runner can compare edited source offsets against older installed copies and reject the block. Use `--config-dir "$EIT_CONFIG_DIR"` on `run_block.py` to select the directory explicitly.

The package installs `config/` and `models/` during `colcon build`. When you pass `config:=$EIT_CONFIG_DIR/controller.yaml`, the launch file loads `joystick.yaml` or `ball.yaml` from that same directory, followed by the selected overlay YAML. Edit the task parameters there, including `joystick_angle_deg`, and restart the controller to apply changes. Rebuild after changing code or launch files. Keep trial output outside the package.

After installing code or ROS message changes, stop the controller and the EIT node with Ctrl+C, then start them again. A running process keeps the libraries it loaded before installation; the recording check will reject a trial if those loaded libraries have since been replaced. `controller.launch.py` can own both processes in one terminal using `with_eit:=true`. Stop any standalone EIT node first so it releases the serial port, then launch the pair, for example:

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=ball with_eit:=true \
  config:=$EIT_CONFIG_DIR/controller.yaml \
  overrides:=$EIT_CONFIG_DIR/ball_y_minus_10cm.yaml
```

This loads `eit.json` beside the selected `controller.yaml` and selects the newest supported EIT model as usual. Use `eit_model:=/path/to/position_model.npz` to pin a model or `eit_config:=/path/to/eit.json` to select another sensor configuration. Ctrl+C stops the pair; if either process exits, the launch closes the other. Re-run the launch command to restart both. `run_block.py` remains a separate operator action; relaunching the pair never queues a robot trial. The default `with_eit:=false` preserves the existing separate-terminal arrangement. If you use separate terminals, stop controller first, then EIT, and restart EIT first, then controller.


Set the actual robot IP and **measured** EE-to-active-face translation/rotation in `controller.yaml`. The supplied EE-to-pad translation is `[0, 0, 0.050]` m, based on the corrected 50 mm measurement. The user confirmed the quarter-turn assembly clocking (pad +X toward EE -Y), and `tool_transform_confirmed` is set accordingly. Measure tool mass/TCP and configure the Franka robot appropriately using your established procedure. The package does not guess payload or change collision thresholds.

The example workspace is positive Base X/Z, centred at Base Y=0. For the extended 102.5 mm joystick shaft, the ready face centre is `[0.45, 0.0, 0.1575]` m, 20 mm above its upright tip. All five joystick landing overlays use this height and physical length. The joystick approach has a 60 mm descent budget; each start caps it at the current measured tip height minus `joystick_approach_below_tip_m: 0.010` m (30 mm descent for an upright shaft). With no live shaft feedback in legacy mode, it uses the upright tip height. `retract_distance: 0.045` m clears the upright tip after a tilted trial. The ball task overrides these ready/approach/retract settings. `landing_offset_x_m` offsets joystick landings along Base X for paired tests; keep `landing_offset_y_m: 0.0` so the task stays centred at Y=0.

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

Because the arm has seven joints for a six-dimensional pad pose, it can reach a pad pose with more than one joint posture. The null-space term applies a soft preference for a reference posture while the Cartesian controller moves the pad. By default, `use_saved_posture: false` captures the joints at controller activation; no posture file or capture step is required. Joint 7 may move away from that reference without a posture-deviation stop. The configured joint-position margins and speed limits still apply to all seven joints.

Only if a specific, previously validated joint configuration must be retained across controller restarts, capture it while the previous controller/driver is at that posture:

```bash
ros2 run franka_eit_vmc capture_posture.py --topic /joint_states --prefix fr3_ \
  --output $EIT_CONFIG_DIR/posture.yaml
```

Stop that FCI driver before running this package. If using the saved posture, add `posture:=$EIT_CONFIG_DIR/posture.yaml` to the launch command. A saved posture is a null-space preference, not a command to move to those seven joint angles. The package cannot guarantee that an arbitrary Cartesian pose is reachable near a chosen q7.

## Physical EIT model and inference

The RBF model is a kernel-ridge position decoder on normalized complex-voltage changes. It uses the selected checkpoint's scan settings, estimates pad XY in millimetres internally, and publishes metres. The runtime reconstructs the RBF weights, width and support limit from the checkpoint's saved training vectors and labels, using the same calculation as the training toolkit. Force labels/force predictions are not used.

The source session is `2026-10-24_11-41-56_JST_d87beb16_david_hydrogel_works` in the parent `eit_vmc_robosoft` directory. Its `evaluation.json` reports 2.79 mm median and 21.61 mm maximum error on 20 held-out touches at previously trained locations. These results do not measure error at unseen locations, in later sessions or during robot motion.

To inspect the newest checkpoint without connecting the sensor or robot:

```bash
ros2 run franka_eit_vmc inspect_model.py
```

The loader uses `allow_pickle=False` and validates the model's saved scan shape and electrode protocol. The node automatically uses the selected checkpoint's electrode pins, drive pattern, frequency, periods and RX gain margin, even when `config/eit.json` lists settings from an older model. It checks the reconstructed measurement order and count before opening the device and logs the effective scan. `config/eit.json` still provides the local serial port and contact threshold. Confirm the saved pin order against the physical wiring before connecting the pad.

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

One `dialout` membership covers both the EIT and ESP32 serial ports when their device nodes are group-owned by `dialout`. A standalone EIT node replaces an older `eit_contact_node.py` that owns the same port when no Franka controller is active. It will not terminate the calibration app, Arduino Serial Monitor, or a sensor used by an active robot controller; close those programs yourself before launching it.

Start live inference:

```bash
ros2 run franka_eit_vmc eit_contact_node.py
```

For standalone sensor checks, request 20 fresh baseline frames with the pad unloaded. This measures the live electrical reference for prediction; it does not train or modify the model. The service returns after calibration completes or reports failure. Joystick blocks call it at each settled ready pose. Ball blocks call it after the operator presses `run` or `r` and before trial approach, so a ball stuck to the pad during ready can be removed first. No separate manual calibration step is needed for `run_block.py`:

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

The offset overlays specify pad-centre displacements in Base coordinates. Current positive offsets are +35 mm and negative offsets are −25 mm along the named axis; the other offset is zero. Confirm each landing stays inside the calibrated sensing region with the real joystick. The block helper checks both active offsets against the selected YAML. Keep the joystick fixture fixed while changing the palm landing.

The joystick pivot remains at Base Z=35 mm. Adding 82.5 mm to the previous 20 mm shaft gives `joystick_length: 0.1025` m and an upright tip at Z=137.5 mm. All five landing overlays put the ready pad face at Z=157.5 mm, with the existing XY offsets. The 60 mm maximum approach budget accommodates a tilted shaft; each trial caps descent using the initial measured tip height plus 10 mm verification allowance (30 mm descent upright). Timeout is 10 s, and the 45 mm retract covers the tip's rise as it returns upright. The software workspace minimum remains at Z=30 mm in the landing overlays.

### 2. Start EIT inference (terminal A; leave running)

```bash
ros2 run franka_eit_vmc inspect_model.py
ros2 run franka_eit_vmc eit_contact_node.py
```

Keep the pad unloaded when starting `eit_contact_node.py`: it automatically collects one baseline from 20 fresh electrical frames after the stream connects and logs completion. If the first connection fails before the 20 frames are collected, it retries after reconnecting. A successful startup capture is not repeated while the process runs. Joystick blocks recalibrate after each settled ready pose. Ball blocks recalibrate after the operator presses `run` or `r`, before approach; remove any ball stuck to the pad and leave it unloaded when pressing the key. To recalibrate a standalone node manually, use:

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

`run_block.py` starts the ESP32 recorder automatically, waits for a sample before any robot motion, and saves `joystick_esp32.csv` beside `block.json` in a dated `center_*` folder. It stops the recorder when the block ends. The robot controller does not read the ESP32. The ESP32 CSV preserves raw ADC counts. The 3.3 V endpoint calibration in `config/joystick_adc_calibration.json` maps ESP32 Y to joystick Base X angle and ESP32 X to Base Y angle using the measured centre and the ±35° stops. `run_block.py` copies this calibration into each new block folder so plots use the matching scale. In terminal B:

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=joystick \
  config:=$EIT_CONFIG_DIR/controller.yaml \
  overrides:=$EIT_CONFIG_DIR/joystick_center.yaml
```

In terminal C, select inward-arc VMC with a 10 mm inward depth for both virtual arcs:

```bash
ros2 run franka_eit_vmc run_block.py --task joystick --block center \
  --mechanism inward_arc --inward-arc-m 0.010 --axis x \
  --repeats 5 --output ~/eit_trials
```

The centre and X-offset block commands below explicitly select `--axis x` for both baseline and EIT conditions. Replace it with `--axis y` to run the other tilt direction at the same landing. The Y-offset example uses `--axis y`. To use the original VMC instead, replace the `--mechanism inward_arc --inward-arc-m ...` options with `--mechanism legacy` and keep the desired `--axis` selection. The controller launch command is the same for either mechanism. Ball blocks do not accept these options.

Joystick blocks now run automatically with this command: the controller moves to ready, approaches, preloads, executes +`joystick_angle_deg`, then −`joystick_angle_deg`, then returns toward zero before retracting and continues to the next condition without `r` prompts or a plotting pause between trials. The script checks the live ESP32 recording before each ready and trial command. Legacy joystick blocks wait up to 20 s for a fresh, stable upright reading near the calibrated X=2043 and Y=1987 raw ADC counts (within 80 counts on each axis). Inward-arc blocks accept a stable tilt, including an ADC-rail reading, and keep the calibrated absolute angle reference. These defaults come from the 3.3 V calibration file; adjust them with `--center-x-raw`, `--center-y-raw`, and `--center-tolerance-raw` if the hardware changes. Legacy blocks stop if the joystick does not return upright. Inward-arc blocks stop if feedback is stale or unstable; reaching an ADC rail does not stop the controller. A controller fault or lost ESP32 stream also stops the block. At the end, the script requests a controller stop, saves the ESP32 recording, and generates the analysis and comparison plots. Use `--manual` to restore the old ready/run prompts. The EIT attachment is initialized at contact and then follows filtered valid EIT positions by default. The baseline assumes the pad centre. Both get the same contact detector, gains, length and angle commands. Review the logged raw sensor position and filtered `attachment_m` in the replay.

Five paired repeats run both conditions, alternating baseline/EIT order across repeats. Automatic inward-arc mode can start the next condition from a stable tilt; legacy mode still requires the joystick to return upright. The script cannot physically reset the object. In manual mode, `q` at a prompt exits the block while the task controller continues holding; Ctrl+C during a child operation requests its bounded stop and ends the block. If auto-detection picks the wrong port, pass `--joystick-port /dev/ttyUSB0` using the actual ESP32 port.

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
ros2 run franka_eit_vmc run_block.py --task joystick --block x_minus \
  --mechanism inward_arc --inward-arc-m 0.010 --axis x \
  --repeats 5 --output ~/eit_trials
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
ros2 run franka_eit_vmc run_block.py --task joystick --block x_plus \
  --mechanism inward_arc --inward-arc-m 0.010 --axis x \
  --repeats 5 --output ~/eit_trials
```

Joystick total: **3 landings × 2 conditions × 5 repeats = 30 trials**. Measure actual angle error over the common command sequence, final angle error, contact losses and completion; measure peak load only with a reliable independent measurement or clearly label the robot wrench estimate. Expect the fixed-centre baseline to degrade more at off-centre landings; centred performance may be similar. These are hypotheses, not established hardware results.

For an additional positive-Y landing, restart the controller in terminal B:

```bash
ros2 launch franka_eit_vmc controller.launch.py task:=joystick \
  config:=$EIT_CONFIG_DIR/controller.yaml \
  overrides:=$EIT_CONFIG_DIR/joystick_y_plus.yaml
```

Then in terminal C:

```bash
ros2 run franka_eit_vmc run_block.py --task joystick --block y_plus \
  --mechanism inward_arc --inward-arc-m 0.005 --axis y \
  --repeats 5 --output ~/eit_trials
```

For negative Y, use `joystick_y_minus.yaml` and `--block y_minus`. The Y overlays mirror the existing X landing offsets: `y_plus` has X=0, Y=+0.035 m; `y_minus` has X=0, Y=−0.025 m. Edit their `landing_offset_y_m` values to adjust the landings. The overlay selects only the landing location; `--axis y` independently selects Base-Y tilt, while `--axis x` keeps Base-X tilt. Both baseline and EIT conditions use the selected landing and trajectory. Including both Y landings adds 20 trials at five repeats, giving 50 trials across all five landings.

### 6. Ball preparation and undisturbed block: 10 trials

Stop the joystick controller. Install the ball/table setup at Base Y = −0.135 m. For the user's 20 mm diameter ball on a base whose top is at Base Z=45 mm, the ball centre is at Z=55 mm and its top at Z=65 mm. `ball.yaml` places the ready pad contact face at `[0.45, -0.135, 0.09]` m, giving 25 mm vertical clearance. With the configured axial 50 mm EE-to-pad offset, the end-effector origin is approximately Z=140 mm at ready. Ball approach descends until contact is confirmed or the pad reference reaches `workspace_min_m` Z, with a 0.1 mm numerical margin. It does not use the joystick's fixed `approach_distance`; the allowed travel follows the Ready height. The ball approach force request is capped at 15 N at the pad, and its timeout includes the nominal descent time from Ready. Ready moves may retain the starting height during horizontal travel before descending over the ball. Mark the ball's starting position and measure its Base-X centre independently. Set `initial_ball_x` in `$EIT_CONFIG_DIR/ball.yaml`; this same value is used for both conditions. Reset the ball to that mark before every trial. The fixed goal is this initial X plus `roll_distance` (default +20 mm), so the commanded roll direction is **Base +X**, from X=0.45 m toward X=0.47 m with the current values. Leave both landing offsets zero in `controller.yaml`.

Ball approach keeps contact verification and workspace checks. Its time limit
adds the nominal pad travel divided by `approach_speed` to `approach_timeout`.
At confirmed contact the bounded incoming command initializes the normal spring.
After `preload_time: 0.55` s and `ball_contact_settle_s: 0.30` s of initial stable
contact, horizontal control engages. Both position conditions then use the
contact-responsive normal mechanism described above. Valid contact loss releases
normal descent and pauses horizontal drive; stale/invalid sensing stops instead.
The same recovery rule applies to baseline and EIT; only their position estimates
differ. Contact return resumes horizontal drive without another 0.30 s settle.

The existing ball values remain `preload: 3.0` N, `normal_k: 800.0` N/m,
`normal_d: 60.0` Ns/m and `normal_force_limit: 4.0` N. After the preload blend,
the unilateral normal spring command is in [-4,0] N. The 500 g tool compensation remains a
separate +4.903325 N at the EE; total requested Z force includes it. Physical
force can differ from these commands. `coupling_k: 3500.0` and `coupling_d: 30.0`
now stabilize the cross-axis direction; `goal_k: 80.0` and `goal_force_limit: 5.0`
act directly along the rolling axis. The 5 N cap was selected for the first
physical run with direct goal force; the older virtual-mass controller used a
different force law. The progress threshold is 5 mm short of the goal and the
dwell is 2.0 s by default. Progress beyond the goal also counts toward dwell.


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

After each ball attempt, enter the **camera-measured signed displacement in cm**
(for example `-8.2` for 8.2 cm toward Base −Y), or `pending` until the video
has been measured. `outcomes.jsonl` stores this independently of the controller
estimate, together with estimated-goal versus time-limit completion and robot
pad travel. Trial faults also offer the measurement prompt before ending the
block. `ball_comparison.csv` puts the two conditions' measured travel and
controller results on adjacent rows. At block end, `vmc_replay/repeat_XX/`
contains an MP4 and PNG for each
baseline and EIT trial; `vmc_replay/ball_vmc_replay.mp4` compares the latest
pair. In new traces, the orange dot shows the EIT sensed surface contact location
when contact and position are valid, including during baseline trials. The
purple ring shows the controller's ball estimate along the task axis; its other
coordinate follows the pad for display. The blue disc is the calculated pad
face, while the black diamond is the robot EE origin reconstructed from the
geometry recorded with that trial. The orange dot disappears
when the sensor position is invalid. Older traces without logged contact X/Y
show the controller estimate in orange; they cannot be reconstructed from
contact flags alone. If the reported actual ball start differs from the
configured start, a green dotted line marks it, a green dashed line marks
10 cm travel from it, and camera final positions use that actual start. The
red dashed line remains the controller's configured absolute goal. The orange
dot is a surface contact, not the physical
ball centre. Neither marker is camera-tracked ground truth. In direct
ball VMC, the purple X is a derived
goal-error point that switches to a hold point during contact/position dropouts;
the moving spring goal shifts this marker further as the near-goal phase
continues. The controller applies a bounded force, not a position command to
that X. New trial plots show the moving spring goal separately from the
unchanged ball stopping goal.
Video generation also runs after a fault when a
partial ball trace exists.

Starting a ball block sends each ready move immediately, without an initial `r` prompt. After ready, remove any ball stuck to the EIT pad, reset the ball to its mark, leave the pad unloaded, and start the independent camera. Type `run` or `r`; the block then recalibrates the unloaded sensor before approach for each condition. `--manual` restores the ready prompt. Do not touch the ball externally during translation. Both conditions keep the centre attachment (`update_ball_attachment: false`). `baseline` estimates progress from initial ball position plus the configured ratio times pad travel (1:1 for sliding); `eit` uses sensed contact position as the filtered progress observation. This tests progress feedback with the same task goal and contact recovery.

### 7. Ball brief-push block: 10 trials

Keep the same controller/configuration running. Establish a repeatable push protocol before this block: use a marked displacement stop or other repeatable jig along the rolling direction, with a consistent externally recorded timing cue during the rolling phase. Choose and record the displacement and timing before collecting results; use them for both conditions. One brief push occurs per trial. Release afterward, preserving palm/ball contact. The controller receives no disturbance flag. Both feedback conditions share bounded contact recovery; camera observation must establish whether physical contact was maintained.

In terminal C:

```bash
ros2 run franka_eit_vmc run_block.py --task ball --block push --repeats 5 --output ~/eit_trials
```

Ball total: **2 physical conditions × 2 feedback conditions × 5 repeats = 20 trials**. Measure camera-derived final goal error, peak disturbance deviation, recovery into a prespecified goal tolerance, and contact loss. Evaluate recovery within the common configured engagement horizon. The baseline assumes that the ball follows the pad while sliding on the base; EIT position feedback is intended to correct errors when that assumption fails.

### 8. Save results and finish

The original five-block procedure produces **50 planned trials**. Additional mechanism comparisons are separate blocks and must not be pooled without their mechanism/depth labels. CSVs are saved under `~/eit_trials/<task>/<block_timestamp>/repeat_XX/`; `block.json` records the block/config check, selected mechanism/depth and both joystick CSV paths and `outcomes.jsonl` records observation status. Joystick blocks save the continuous ESP32 angle CSV and write `pending_esp32_review` after each completed trial. In automatic mode, plotting waits until the block has ended and the controller stop has been requested, so plotting cannot interrupt the next trial. The runner saves `joystick_<condition>_<timestamp>_analysis.png` beside each trace, `repeat_XX/comparison.png` for paired repeats, and `block_comparison.png` in the block folder; it also plots a partial attempt after a fault. The comparison plots overlay desired joystick angle from the controller and actual joystick Base X angle from the ESP32, with actual-minus-desired error in the paired plot. For blocks with palm-orientation telemetry, `block_comparison.png` also adds a 3D error panel under each repeat: zero means the palm face is perpendicular to the measured joystick shaft. New traces also record actual and desired pad-face normals in Base coordinates and produce `repeat_XX/orientation_comparison.png`. That plot compares desired and actual palm tilt against the measured joystick shaft, plus the full 3D pad-normal error from perpendicular alignment. A zero alignment error means the pad face is perpendicular to the shaft; stale samples are omitted, while ADC-rail angles appear as bounded estimates. Older traces lack pad normals, so they cannot produce this orientation plot. Each trial analysis plot overlays desired and measured Base X angle in the joystick-angle panel and also shows measured point and reference, controller requested force, Franka estimated external force in Base X/Y/Z, and the EIT signal/contact decision. The external force estimate is reported at the robot stiffness frame and does not isolate joystick contact. Joystick angle uses piecewise linear interpolation between centre and the measured ±35° stops; readings at the ADC rails are approximate. A red line marks the fault time. The EIT panels show signal versus cutoff, contact and freshness flags, and pattern distance versus model support limit; the fault message also prints these values. A positive signal can still produce no contact if its pattern is outside model support, its predicted point is outside the pad, or the contact hysteresis has not engaged. Older CSVs lack these EIT diagnostic fields, so replotting them can mark the fault but cannot recover the missing EIT evidence. Older blocks without a copied calibration still plot raw ESP32 counts. Pending status does not claim object success. The plots can be regenerated with `ros2 run franka_eit_vmc plot_joystick_results.py <block-folder>`. Ball blocks still ask for a camera-based observation. Record unsuccessful attempts and faults; if rerunning a failed block, keep the original attempt files and identify the additional trials explicitly.

Controller `done` means its sequence finished, not object success. New controller CSVs include received calibrated joystick angle and its validity/freshness; camera ball ground truth is still external. The continuous calibrated joystick CSV is saved for both mechanisms. Define the success tolerance before examining results. Plot joystick errors by offset/condition and ball final errors/recovery traces by disturbance/feedback condition.

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

## Reproducing recorded experiment conditions

Both joystick and ball runs now save a `reproducibility/` directory inside the
record folder. This applies to `run_block.py` and standalone ready/trial calls
through `experiment.py`. Rebuild the package and restart the controller, EIT
contact node and joystick feedback node to enable the new recording interfaces.
An older running controller is rejected before the next recorded motion because
its exact build inputs cannot be recovered from today's checkout.

```text
<block-folder>/
  block.json
  repeat_01/<task>_<condition>_<timestamp>.csv
  reproducibility/
    manifest.json
    commands.jsonl
    parameter_events.jsonl
    snapshots/
      <trace>.before_motion.json
      <joystick-ready-trace>.after_ready_calibration.json
      <trace>.after_operation.json
      <same-name>.yaml
    files/<sha256>/<original-filename>
```

Each snapshot records **every declared controller and EIT-node parameter** with
its ROS type, the current controller status/contact source, and the action's
command line. This includes joystick/ball trajectories, timings, all gains and
limits, geometry, posture and mechanisms. Parameter events preserve changes
during the operation, including temporary EIT threshold changes.

The sensor snapshot contains the exact loaded model bytes and hash, all 20 raw
unloaded baseline frames, baseline timestamp, contact hysteresis, threshold,
effective electrical protocol, firmware version, serial port and per-channel
autogain/applied gains. The joystick feedback snapshot contains its loaded ADC
calibration, centre overrides, decoder/filter settings and runtime state.

The controller embeds an archive of its own build inputs: dirty and untracked
package source, configuration, launch files, Python tools, messages, models,
tests, Git revision/diff and compiler configuration. Recording copies that archive
out of the running controller, plus its executable and loaded Franka/controller
libraries. It also preserves actual launch parameter files, environment/package
versions, SDK filter/rate-limit settings and startup robot tool/load masses,
inertias, centres of mass, transforms, joint posture and server version. Source
at build time and configuration at recording time are recorded separately.
Executable/library capture currently requires the experiment nodes on the same
Linux host as the recorder. Boot/process identity is checked so a remote node's
PID cannot silently identify an unrelated local process.

Artifacts are copied and addressed by SHA-256; later edits to source, models or
configuration do not overwrite earlier artifacts. JSON is the authoritative
record. Adjacent YAML files are convenient ROS parameter inputs; they do not
restore a calibrated sensor baseline or choose a service-based contact condition
by themselves. Use the recorded `experiment.py --condition ...` selection and
matching source build when reconstructing a run. Do not execute archived binaries
on a different machine as a substitute for rebuilding the recorded source.

A failed required snapshot prevents the next motion and leaves an `incomplete`
snapshot with its error. Final snapshots are attempted after completion, faults
and interruption. None of this recording performs disk I/O in the robot torque
callback. It does not repair incomplete historical records.

Physical conditions that cannot be read from software can be supplied without an
extra prompt, for example ball dimensions/material, fixture measurements, pad
condition, camera settings, firmware identity and push-jig settings:

```bash
ros2 run franka_eit_vmc run_block.py --task ball --block push \
  --conditions-file /path/to/physical_setup.json --output ~/eit_trials
```

The supplied file is copied unchanged. The same option works for joystick blocks
and standalone `experiment.py` calls. A missing file stops the run before motion;
omitting it is explicitly marked in the manifest. For standalone calls,
`--record-root <folder>` can group their snapshots into a common record folder;
block runs set this automatically.

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
