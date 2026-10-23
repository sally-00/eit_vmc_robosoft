# UR3e weighted EIT position collection

## Guided manual contact-position training and testing

`contact_position_app.py` adds a standalone desktop workflow for manually poking the pad. It uses the existing Amodo serial reader and does not connect to or move the robot. It learns **X/Y contact position only**, without force labels. The existing weighted robot collector remains available below. The model code in `../../ur3e_sim/src/eit.py` and the joystick simulation was trained on synthetic signals; it is not a trained model for your physical pad.

The view renders the **actual engraved STL**, including the notch and numbers. The default is the counterclockwise plate: **1 bottom, 3 right, 5 top, 7 left**, viewed from the engraved side (+Z). The notch lies between 4 and 5. Align your physical plate with this view. Use `--numbering cw` for the older clockwise STL, where 1 is at the top and 5 is at the bottom. Upright numbers outside the mesh repeat the engraved labels for readability. Coordinates and learned labels are in this same pad frame, with X right and Y up.

Install the requirements plus your system's Tk package (`python3-tk` on Ubuntu/Debian). Try the complete interface without hardware:

```bash
python3 contact_position_app.py --demo --points 20 --repeats 3
```

In demo mode, click the pad to hold a synthetic touch and right-click (or select **Demo: release**) to release it. These signals exercise the interface and are not a calibrated EIT simulation. Demo models cannot be loaded in hardware mode.

**Training model:** Select **Training model → rbf / mlp** in the window before collecting, or after collection before selecting **Train model**. RBF remains the default, set by `eit_training_model: "rbf"` in `config.json`. Override the initial selection at launch:

```bash
python3 contact_position_app.py --training-model mlp --points 20 --repeats 2
```

Both models use the same normalized complex-voltage changes and predict X/Y position in millimetres. For the current 304-measurement scan:

| Model | Structure and fitting |
| --- | --- |
| `rbf` | 608 inputs → Gaussian similarity to each training touch → 2 outputs; kernel ridge regression |
| `mlp` | **608 → 128 → 64 → 2**; tanh hidden layers, linear X/Y output; backpropagation with Adam |

The MLP runs in NumPy with the existing requirements. During optimization, input vectors are scaled by the square root of their feature count and X/Y targets by the active pad radius; predictions are returned in mm. It uses seed 21, batches of up to 64 touches, learning rate 0.003 and L2 weight regularization 0.00001. Training runs for up to 1,000 epochs and stops early if training RMSE per coordinate drops below 0.1 mm. This training-fit criterion is not an accuracy guarantee. Saved metadata/evaluation records the architecture, optimizer, completed epochs and training RMSE. Held-out evaluation excludes the entire final touch repetition, trains a separate model on the earlier repetitions, then reports position errors; the saved model uses all collected touches.

To compare both models from one collection, train one, change **Training model**, then select **Train model** again. This remains available after training and after stopping a live test while that session's records are still in memory. RBF saves `position_model.npz` and `evaluation.json`; MLP saves **`position_model_mlp.npz` and `evaluation_mlp.json`** in the same session folder. Switching model type preserves the other model's files. Training the same type again replaces that type's files, and the main `manifest.json` describes the most recently fitted model. Loading a model clears in-memory training records and restores its saved model type; each model file contains its own complete metadata. Older RBF models remain loadable. MLP files use model format version 2 so older software cannot interpret them as RBF models. Weak-signal rejection stays disabled, unfamiliar predictions remain visible, and both live strength/familiarity readouts work with either model. Compare held-out results and separate unseen physical locations to assess accuracy; a small dataset can overfit an MLP.

**Device power and USB connection (Amodo firmware v1.1.1):**

1. Power the EIT from **24 V DC through its barrel jack using the supplied MEAN WELL adapter**. The blue **POWER** LED indicates that 24 V power is connected. The firmware instructions require this external supply; a USB connection alone is not the documented power arrangement.
2. Connect the computer to the USB-C connector labelled **USB - HS** using a USB data cable. The connector labelled **USB - FS is unused** by this firmware. The separate older hardware note about intermittent USB-FS connectivity is not the startup guide for this connection.
3. Check **STAT2 (green)**: it lights when USB has enumerated. Linux should then expose a serial port, normally `/dev/ttyACM…`. If STAT2 stays off, troubleshoot the power, USB-HS cable and computer connection before assigning a serial port.
4. Leave the pad untouched and select **Connect (pad untouched)** in the trainer. The program configures the measurement scan and starts streaming; no manual START command is required. **STAT1 (red)** indicates activity while processing commands, finding gains or streaming.

The **F1** and **F2** buttons both reset the device; neither is a required measurement-enable button. Normally the application controls measurement startup. These details come from the **Usage** and **Status LEDs** sections of the root `README.md` inside `../../Amodo Files/Firmware/ARIA_EITSYS_FW-v1.1.1.zip`.

**Connection on this computer:** The currently detected EIT is `/dev/ttyACM1` (STMicroelectronics `0483:5740`, serial `376D395E3033`). `config.json` uses the persistent path `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_376D395E3033-if00`, which was verified to open successfully with exclusive serial access. This avoids depending on the current `ttyACM` number. A different EIT unit needs its own path or `eit_port: null` for automatic detection. The reader accepts bundled v1.1.x firmware and the previously verified v1.2.1 protocol; unverified future versions are rejected. The trainer reads its configuration at startup: close and restart it after changing `eit_port`.

**Immediate permission for the EIT USB port:** If the trainer or IDE reports `Permission denied` for `/dev/ttyACM0`, grant your user read/write access directly to the current EIT device:

```bash
sudo setfacl -m u:birl:rw /dev/ttyACM0
```

Enter your Linux password in your terminal when prompted, then retry **Connect (pad untouched)**. This grants access to `birl` and takes effect for already running applications. The permission applies to the current device node; unplugging or re-enumerating USB can remove it, so rerun it if needed. The currently identified EIT is `/dev/ttyACM0`; check the actual port if the device number changes.

**Serial access through group membership:** The account `birl` is already a member of `dialout`, but the current desktop session has not picked up that group. Enable the existing serial-port permission in your terminal with:

```bash
newgrp dialout
```

Then close the existing trainer and launch a new instance from that terminal:

```bash
cd "/media/birl/Yi Cam data/09_EIT_vmc/ur3e_weight_position_data_collection/ur3e_code"
/usr/bin/python3 contact_position_app.py --points 20 --repeats 1
```

`newgrp` starts a shell with the updated group; it does not change the permissions of an already running IDE or trainer. Alternatively, launch just the trainer with serial access enabled from this folder:

```bash
sg dialout -c '/usr/bin/python3 contact_position_app.py --points 20 --repeats 1'
```

After logging out and back in, normal `python3` launch should have the updated group membership. The `sg` command above avoids needing to log out now.

An earlier hardware connection check received five frames with 208 measurements each on firmware v1.2.1, then stopped the stream. Each frame had 2–3 clipped channels; those diagnostic frames were not saved as training data. The clipping option below controls acceptance of newly collected training data.

**`--demo` versus hardware:** With `--demo`, the app generates synthetic signals and never opens a USB connection. Without `--demo`, **Connect** configures and reads the real Amodo EIT device. Both modes use the same point guidance, configurable rounds, debug plots, training and testing interface. Both follow the configured electrode count (currently 16). Demo and hardware models are kept separate.

The current configuration uses **16 electrodes, device pins 1–16**. `eit_electrode_pins` must list the device connector pins in **physical order around the pad**. It is currently `[1, 2, ..., 16]`; reorder this list if the wiring is not sequential around the pad. The STL's engraved 1–8 labels are mechanical orientation landmarks, **not a map of the 16 electrodes**. Set `eit_port` if auto-detection does not find exactly one Amodo device. Override the pins and port with `--pins` followed by 8 or 16 pin numbers and `--port /dev/ttyACM0`. For the reported sequential 16-pin setup:

```bash
python3 contact_position_app.py --points 20 --repeats 1 --pins 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
```

**Drive pattern and measurement count:** The current configuration selects `eit_drive_pattern: "combined"`. For 16 electrodes, this uses **304 measurements per frame and 608 model input features**. The first 208 measurements retain the original adjacent-drive scan. The next 96 use the eight opposite drive pairs **1–9, 2–10, 3–11, 4–12, 5–13, 6–14, 7–15, 8–16**, with 12 non-overlapping adjacent voltage-sensing pairs per drive. Channels **293–304** use the **8–16** current drive. All channels still use adjacent voltage sensing. Outputs remain X/Y position in mm.

| Current drive pattern | 16-electrode measurements/frame | Model inputs |
| --- | ---: | ---: |
| `adjacent` | 208 | 416 |
| `opposite` | 96 | 192 |
| `combined` (current default) | 304 | 608 |

For 8 electrodes, these patterns produce 40, 16 and 56 measurements respectively. Choose **Current drive pattern** before connecting, or select it explicitly at launch:

```bash
python3 contact_position_app.py --drive-pattern combined --points 20 --repeats 1 --allow-clipped
```

Close any old trainer window and restart after this update: an already running process retains its original scan. The top of the window shows the selected pattern, electrode count, measurements/frame and model input count before connection. A combined 16-electrode session should show **304 measurements/frame**. This scan was verified on the connected v1.2.1 device with five complete frames and 304 loaded gain configurations, then stopped. **Start new training for the combined scan**: an existing 208-measurement model cannot use the additional measurements. Loading an older model before connecting restores its original adjacent scan and displays 208 again; this preserves compatibility with its training data. Loading a model with a different scan after connecting is rejected. Session and model metadata store the drive pattern and exact drive/sense order.

```bash
python3 contact_position_app.py --points 20 --repeats 3
```

1. Match the numbered plate orientation, remove all contact, and select **Connect (pad untouched)**. Gain setup runs with the pad untouched.
2. Set **Training points** (default 20) and **Touches / point** (default 3) and select **New training**. The centre and distributed disk targets appear, inset 5 mm from the active boundary.
3. For each point, lift off fully and select **Capture untouched baseline**. The orange target stays hidden during this step and appears after the baseline capture finishes. Then touch the orange cross, hold still, and select **Capture held touch**. Keep holding until the capture finishes. The target disappears for the next untouched baseline; release fully before recording it. A complete round visits every location before the next round starts. Repeated independent touches help capture placement and pressure variation; vary pressure moderately across rounds while keeping the same tip/contact style.
4. Select **Train model** when all touches are saved. The final repetition is held out for an error report at the trained locations, then the saved model is fitted using all touches. With one repetition there is no independent validation.
5. Select **Live test**, release the pad, and capture the fresh test baseline. Poke the pad: the pink dot shows the estimated contact centre. Weak-signal/contact-strength checks are disabled, so the dot can remain visible after release. Unfamiliar signals still show a predicted dot and X/Y coordinates, with “unfamiliar signal” in the status. The dot clears on a rejected/error capture. Select **Stop test** to recapture the baseline after drift.

**Allow clipped readings:** This is currently enabled in `config.json` (`eit_allow_clipped: true`) for the requested experiment. The **Allow clipped readings** checkbox accepts flagged measurements during connection, untouched baseline capture, touch capture, model fitting and live prediction. You can leave the normal launch command unchanged, or explicitly enable it with:

```bash
python3 contact_position_app.py --points 20 --repeats 1 --allow-clipped
```

Uncheck the option before a new session or launch with `--no-allow-clipped` to reject flagged measurements again. The selection is fixed during training collection and active testing. Readings are used as reported, and the original clipping flags remain in the saved frames and debug plots. Session/model metadata records `allow_clipped`; the evaluation also counts `clipped_frames`. A loaded model uses its saved clipping choice for direct model predictions; the GUI's checkbox controls its live captures and predictions. Accepting clipped values may reduce position accuracy. Empty captures, non-finite/negative values and inconsistent channel counts are still checked.

**Weak-signal checks removed:** Touch captures are saved and fitted without testing their strength against untouched baseline noise, including captures with no change from the baseline. Live prediction also ignores contact-strength thresholds, including thresholds stored in older models; no retraining is needed to disable that check. The pink dot may remain visible after release because signal strength no longer determines contact/no-contact. Unfamiliar signal patterns also produce position estimates, as described below. New model/session metadata and evaluation reports record `contact_strength_check: false`; the legacy model threshold field is saved as zero for file compatibility.

**Zero voltage readings:** A single zero reading no longer rejects an otherwise usable capture. Zero touch voltages are allowed. A channel whose mean complex baseline is zero/near-zero (magnitude ≤ 1e-12 V) cannot be used for relative-voltage division. During fitting, the trainer excludes any such channel across all collected baselines and uses that same channel selection for every training touch and prediction. The original raw values stay unchanged. The fixed feature array retains its original size, with excluded real/imaginary components set to zero so they do not affect model distances. The saved model and updated session manifest include `usable_channel_mask` and `usable_measurement_channels`; the evaluation lists excluded channel numbers. Models from before this change retain all their original channels. A new test baseline must still supply the model's required channels; if a required baseline channel is lost, recapture the baseline or retrain. An entirely zero capture remains an error because it contains no measured signal.

The voltage debug plot marks channels containing zero readings with orange circles and lists their channel numbers. Incomplete/invalid serial frames are discarded while waiting for the requested number of complete frames within the capture timeout; they are not padded or used as training examples. The discard count is preserved in the capture metadata and debug status. The reader requests exclusive serial access. Close other EIT trainers or serial viewers before connecting; a second reader can interfere with the USB stream.

**Live signal debugging:** The right-hand panel updates automatically after connecting, including while you are finding the next touch point. It shows all configured measurement channels (304 for the current 16-electrode combined scan): raw voltage magnitude in mV RMS, phase in radians, real/imaginary complex-voltage change relative to the captured untouched baseline (percent), and a rolling 30-second RMS-change trace. Dashed lines show the baseline. Channel numbers identify the drive/sense configurations in the saved manifest, not the electrode numbers. The plots average the frames in each fresh acquisition; they are not an oscilloscope waveform of the 50 kHz carrier. The actual update rate depends on the device's frame rate and frames per capture. The combined 304-measurement scan can take longer than the adjacent-only 208-measurement scan.

The debug status identifies preview versus button-triggered captures, the number of averaged frames, reading age, and clipped-channel count; red crosses mark clipped channels on the voltage plot. Clipped readings remain visible even when **Allow clipped readings** permits them for training and prediction. With that option off, clipped captures are rejected. Failed reads clear the current curves and show an error. Capture a baseline to enable the relative-change plots; the comparison trace restarts with each new baseline. Uncheck **Live signal debug** to pause background preview; explicit captures and live model testing still update the plots. Preview does not save a training touch or advance a point. Only the capture buttons save training data. Background preview and saved captures share the same serial connection, with reads serialized so they cannot overlap. Capturing can wait for an in-progress preview acquisition to finish.

**Live prediction readouts:** After selecting **Live test** and capturing the untouched test baseline, two readouts below the signal plots update with every position prediction, even with **Live signal debug** unchecked. **Signal strength** shows the RMS complex-voltage change from that baseline as a percentage over the model's usable channels, plus the combined dimensionless strength score. For example, equal 1% relative changes across all usable channels give 1% RMS change. Excluded zero-baseline channels do not contribute. **Signal familiarity** shows the distance to the nearest normalized training example and the model's familiarity limit: lower distance means a closer pattern match; distance at or below the limit is labeled familiar. This is not a confidence percentage or contact/no-contact decision. A predicted position outside the pad is identified separately. These readouts never reject weak or unfamiliar signals. Stopping testing, starting a new baseline/session, loading a model or failing a prediction clears the readouts so old scores are not presented as current. Existing saved models work without retraining.

The point count and repetitions can also be changed in the window. `--radius-mm` defaults to `active_radius_m` in `config.json` (45 mm); measure your active area. `--margin-mm` controls the inset (default 5 mm). Reserve enough inset for the contact tip footprint. `--preview layout.png` saves the numbered plate and target plan without connecting to hardware.

Each training session creates a unique folder in `contact_position_runs/`, named `YYYY-MM-DD_HH-MM-SS_JST_<id>` using the session start date and time in Japan (Asia/Tokyo). For example, `2026-10-23_16-42-53_JST_05c7c837` starts at 16:42:53 JST on 23 October 2026. The short ID prevents collisions between sessions started in the same second. The folder contains `manifest.json` (settings, target locations, numbering, device/gain metadata, `created_utc`, `created_local` and `session_timezone`), `touches.jsonl` (raw contact and baseline frames with host timestamps), the selected model/evaluation files described above. Existing folders with compact UTC names remain loadable; the `Z` in those names means UTC, nine hours behind JST. Interrupted sessions preserve already saved touches but are not automatically resumed. Reload a model through **Load model…** or:

```bash
python3 contact_position_app.py --model contact_position_runs/YOUR_SESSION/position_model.npz
```

The model restores its numbering, radius, pin mapping, drive pattern, frequency and measurement periods; it requires a fresh untouched baseline before testing. Loading a model after connecting rejects incompatible measurement settings. Close other serial tools before connecting.

The estimator uses normalized complex voltage changes with the selected RBF or MLP position regressor. Unfamiliar signals are flagged in the status but still return the raw predicted X/Y coordinates and show the pink dot. Predictions outside the active pad radius are also returned without clamping. These predictions can be inaccurate, including when the pad is untouched. This applies to loaded older models without retraining. New model/session metadata and evaluation reports record `unfamiliar_predictions: true`. Normalization reduces simple pressure scaling, but real pressure, contact shape, hysteresis and baseline drift can still change estimates. The dot is a predicted single contact centre, not an EIT reconstruction or measured contact area. Twenty locations are a starting layout, not a measured accuracy guarantee. The repeated-touch error report does not assess unseen locations or later sessions; check separate physical positions before relying on interpolation. No physical-pad validation has been performed by the offline tests.

Offline checks: `python3 -m unittest test_contact_position -v`. GUI workflow checks run when a graphical display is available; model/signal checks always run.

This folder also records static EIT measurements at known positions with a weighted robot collector. The shared serial reader supports either 8 or 16 electrodes; the current configuration uses 16. The UR3e moves the [R3 deadweight attachment](../ur3e_weight_position_data_gather_attachment/R3/README.md) with slow Cartesian `movel` commands. One invocation records one weight configuration in a unique run directory. `eit_reader.py` reads the Amodo ARIA EITSYS device through USB serial, following the Python examples and firmware in `../../Amodo Files/Firmware/ARIA_EITSYS_FW-v1.1.1.zip`. `--metadata-only` explicitly runs a motion check and writes null EIT fields.

The collector imports the read-only RTDE transport, Dashboard checks and primary-interface script upload from `../../../00_VMC_demo_ur3e/ur3e_demo/rtde.py`. It does not run that demo's torque or force controller. The robot must be a UR3e in Remote Control with a correctly configured active TCP and payload. All robot coordinates are metres in the UR Base frame; pad coordinates are relative to the calibrated center.

## Move to a pose for mounting the attachment

The figure below shows the six R3 printed parts in their **assembled** positions and an exploded view. The gray cylinder represents the UR3e wrist at the tool flange; the printed part geometry comes from the R3 CAD. Attach the round plate of **01 fixed frame** to the robot tool flange with four M6 bolts. The bridge then extends sideways to the vertical guide; the weight cage stands above it and the contact tip points down. See the [full-size PNG](figures/r3_ur3e_attachment_assembly.png) or [PDF](figures/r3_ur3e_attachment_assembly.pdf) for a larger view. The STL files are oriented for printing, so their file orientation is not the installed orientation.

![R3 printed parts assembled on a schematic UR3e wrist, with exploded assembly view](figures/r3_ur3e_attachment_assembly.png)

**Part 01 is asymmetric.** At the revised mounting pose, place its flat round plate against the UR3e tool flange and hold the frame so its bridge runs toward **Base +Y** and its guide block sits at the far end of that bridge. The optional pin clearance slot in the round plate is then **above** the plate center; the cage will stand up and the plunger tip will point down. The view below shows the robot-facing side of part 01 in this orientation. The slot is clearance only; it does not locate the plate precisely.

![Part 01 clocking at the proposed UR3e mounting pose](figures/r3_part01_mounting_orientation.png)

Align all four M6 holes without forcing the printed plate or the robot's locating pin. Install the four M6 fasteners and washers as specified in the [R3 hardware instructions](../ur3e_weight_position_data_gather_attachment/R3/README.md#hardware---metal-fasteners-only), and check the actual bolt projection into the UR flange. If the holes or pin clearance do not fit with the bridge in this direction, stop: the proposed wrist rotation does not match the physical flange clocking. Teach a different **bare-flange** mounting pose with the pendant and record it with `python mount_pose.py record`, then measure the mounted tool transform before scanning. Do not turn part 01 a quarter-turn and assume the current TCP and tip coordinates still apply.

The revised **bare-flange pose** is `(x, y, z) = (0, -0.4, 0.3)` m in UR Base, with rotation vector `(rx, ry, rz) = (-1.2092, -1.2092, 1.2092)` rad. Its orientation is 180 degrees about **UR Base Z** from the original target, and its flange is 0.2 m farther along Base `-Y`. The tool0 axes point `X` toward Base `+Y`, `Y` down, and `Z` toward Base `-X`. Mount the R3 plate so the bridge from its flange toward the guide (CAD `+X`) points toward Base `+Y`, and its cage points up (CAD `+Z` toward Base `+Z`). The CAD tip is then nominally at `(0, -0.31, 0.25)` m. This is a mounting location above the pad region, not the pad contact pose. The approximate CAD envelope reaches to about 0.56 m above Base. The original pose reached on the physical robot lacked cage space; the revised pose has **not** been checked for physical clearance. Check the wrist, full cage and surrounding fixtures before attaching the assembly. With this mounting clocking, a **nominal** guide-axis TCP at CAD height `z=-50 mm` has tool0 offset `[0.09, 0, 0, 0, 0, 0]` and probe direction `[0, 1, 0]` in the TCP frame; measure the assembled fixture before entering these in `config.json`.

The proposed pose is reachable in the bundled UR3e model. The model does not establish clearance for the physical installation. `config.json` contains `robot_ip: 192.168.1.200` from the previous demonstration; verify the address on the robot. With the R3 attachment **off**, inspect the proposal:

`move` sets the robot's active TCP to the bare flange (all six offsets zero) through a short URScript command if needed, then verifies the offset through RTDE. This changes the active robot TCP even during inspection, but does not move the robot without `--execute`. It stops an existing robot program before setting the TCP. If the R3 attachment is still installed, remove it before running this command. The 20.3 mm TCP from the earlier torque demo is for another tool; set the measured R3 TCP after mounting the new attachment.

```bash
python mount_pose.py move
```

`move` no longer needs `mount_pose.json`. It prints the proposed flange and tip positions and the difference from the current robot pose without motion. The proposed move can now start from any stationary pose with an inverse-kinematics solution. It follows a joint-space `movej` path, so a large orientation change can sweep the wrist and arm through a wide region. Inspect the full path with the attachment off before executing:

```bash
python mount_pose.py move --execute --path-clear-verified
```

The robot controller checks inverse kinematics with the current joint angles as its reference, following the [URScript inverse-kinematics interface](https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-scriptmanual/G5/get_inverse_kin_has_solution.htm). It uses joint position control at 0.1 rad/s and 0.2 rad/s², monitors measured joint speed and final TCP pose, and sends Dashboard stop on an error. The timeout is 180 seconds to accommodate a longer joint-space move. Once the robot is stationary at the mounting pose, support and attach the R3 assembly. The collection scan uses a separate calibrated tool TCP and begins only after mounting and pad calibration.

If this proposal conflicts with the physical installation, `python mount_pose.py record` still stores a pendant-taught bare-flange pose, and `python mount_pose.py move-saved` inspects it. `move-saved --execute --path-clear-verified` returns to that custom pose from nearby.

## Mount and measure the R3 fixture pose

The CAD assembly uses its own axes. Its flange center is at `(-90, 0, -50)` mm and its tip is at `(0, 0, -100)` mm in those axes. The robot flange face is perpendicular to CAD Y; the probe slides along CAD Z. The 90 mm transverse offset is built into the fixed frame. The assembled tool can be clocked around the flange, so these CAD coordinates alone are **not** a UR TCP setting. Measure the actual mounted tool transform and enter it in `config.json` as `expected_tcp_offset_pose`. Put the TCP origin on the **fixed guide axis**, at a convenient reference height. The moving tip is free in Z and cannot serve as a rigid TCP. Verify the probe center in XY against a physical mark after setting the TCP.

Enter `probe_axis_in_tcp`, the unit vector pointing from the cage toward the pad, expressed in the active TCP frame. Derive it from the mounted CAD direction `(0, 0, -1)`, including the actual mounting rotation and the TCP rotation. Check it by holding the pad level and viewing the guide from two perpendicular directions. The program rotates this vector into Base and requires it to point down within 2 degrees at every calibration and scan pose. It also checks that the robot flange normal is perpendicular to the probe axis, as in the R3 CAD, and requires the active TCP offset to place the guide axis 80–110 mm transversely from the flange center. These checks catch a tilted or incorrectly entered pose; they cannot prove that the wrist, 310 mm tall fixture, cage, cable, weights and nearby objects have collision clearance. Inspect the entire path at the loaded mass, including the full 15 mm lift and the outer scan positions. The `--fixture-clearance-verified` option records that inspection as an operator action.

Weigh the entire moving head and each set of added weights. Set the active UR payload and center of gravity for the mounted assembly, then enter those values in `expected_payload_kg` and `expected_payload_cog_m`. Update these values when the weight changes. The nominal normal force label is `(moving head mass + added mass) × 9.81 N`, provided the shaft slides freely and no other part carries the load. Perform the scale checks in the R3 README before treating this label as measured force.

Once the values are entered, place the robot in a stationary trial pose and run `python collect.py inspect-pose`. This read-only check reports the Base TCP pose and probe tilt, and rejects a TCP or payload mismatch. Do it before recording rim marks and after changing the weight.

## Find the pad center

Mount the R3 attachment on the robot and set its measured TCP, probe direction, and payload in `config.json` first. Fix the pad to a level, rigid backing. The adjacent simulation models a 90 mm active disk inside a 130 mm device; measure the physical pad's active diameter and update `active_radius_m` if it differs. Mark four opposite points on the active boundary: east, west, north, and south. These are marks on the pad, not robot coordinates.

Record **one mark at a time**. Start `python collect.py calibrate east`; the program waits at a prompt. While it waits, jog the robot **manually with the pendant** until the fixed guide axis is vertically over the east mark. Keep the moving tip and the rest of the attachment clear of the pad, stop the robot, and press Enter. The program then opens a read-only RTDE connection, checks that the robot is stationary and the attachment points down, records the current TCP pose, and exits. Repeat the same sequence for west, north, and south, holding the same robot orientation and clearance height. No `calibrate` command sends a motion command.

```bash
cd '/media/yi/Yi Cam data/09_EIT_vmc/ur3e_weight_position_data_collection/ur3e_code'
python collect.py calibrate east
python collect.py calibrate west
python collect.py calibrate north
python collect.py calibrate south
```

The software averages opposite rim points to calculate the center. It checks that the two midpoints agree within 3 mm, the marked radius matches `active_radius_m` within 5 mm, the rim poses have similar height, and the axes are nearly perpendicular. `pad_calibration.json` stores the measured Base-frame center and pad axes. If the pad, flange mount, active TCP or fixture moves, start a new calibration file.

For the fifth mark, start `python collect.py calibrate contact` and use the pendant to move the mounted attachment over the calculated center while the program waits. With the shoulder carrying the moving head, descend slowly until the tip is supported by the pad. Lower the fixed guide about **2 mm farther**, leaving a visible shoulder gap as specified by the R3 design. Stop the robot, then press Enter to record the **guide TCP pose** at this measurement height. The program does not perform this descent.

```bash
python collect.py calibrate contact
```

Keep the four rim captures at least 3 mm above this contact reference. `contact` defines the robot's guide height for the entire scan; it is not a measurement of the floating tip's Z position. Inspect shoulder clearance at the center and edge positions. Recheck on the scale that a 2 mm guide gap does not change the load reading. The default 15 mm travel lift should recapture the shoulder and lift the tip fully clear before sideways motion; verify this visually on the assembled tool.

## Inspect and collect

Fill the `null` values in `config.json` from the actual robot and tool. In particular, do not copy the 20.3 mm TCP offset or 0.1 kg payload from the torque demo: those describe another tool. The default `active_radius_m` value is 45 mm from the simulation model. The grid reserves 5 mm for the contact face radius plus `edge_margin_m`, so the complete 10 mm flat stays inside the active disk. Plan the positions without connecting to the robot:

```bash
python collect.py scan --weight-id W000 --added-mass-kg 0 --moving-mass-kg 0.250
```

A physical motion check, once calibration and fixture clearance are confirmed:

```bash
python collect.py scan --weight-id W000 --added-mass-kg 0 --moving-mass-kg 0.250 \
  --execute --metadata-only --fixture-clearance-verified
```

For a data run, install the dependencies with `python3 -m pip install -r requirements.txt`, verify `eit_electrode_pins`, and remove `--metadata-only`. The pin list gives the 8 or 16 **device connector pins** in physical order around the pad; check the actual wiring or continuity. The Amodo hardware notes include an older board revision with misleading electrode silk labeling. `eit_port` may name the serial device; when null, the reader requires exactly one USB device with the Amodo example's `0483:5740` VID/PID. The host uses the firmware's `RESET`, `FREQKHZ`, `SETPERIODS`, `MODE phase`, `LOAD`, `AUTOGAIN`, `DUMP` and `START` commands. The default 50 kHz and one period follow the Amodo live-view example and can be changed in `config.json` after checking signal quality.

The shared reader follows `eit_drive_pattern`: adjacent uses N × (N−3) configurations; opposite uses (N/2) × (N−4); combined appends both groups. For the current 16-electrode combined configuration this is 208 + 96 = 304 measurements per frame. Every measurement uses an adjacent sense pair that does not share either drive electrode. Every frame preserves the raw serial line, all voltage magnitudes in Vrms, all phases in radians, and a clipping flag for each measurement. The run manifest stores their exact drive/sense order, electrode/channel counts and the device's post-autogain gain dump. The firmware stream has no device frame timestamp, so each frame has a host receipt timestamp. At each grid point, the collector takes five fresh frames at the raised no-contact pose and five after the contact settle time. The reader discards frames queued during motion. Repeat the command for each weight with a distinct `--weight-id`; the collector makes a unique timestamped directory under `runs/` and never resumes or overwrites an interrupted run.

Automatic robot motion begins only with `scan --execute`. Before that command, use the pendant to put the robot at the **center travel pose** (`center_contact_tcp_pose` raised by `lift_m`). Each position then follows: horizontal `movel` at travel height, vertical `movel` to the calibrated measurement height, settling and acquisition, then vertical `movel` back to travel height. The script uses at most 5 mm/s and 20 mm/s² by default. It checks the actual TCP after each command, RTDE safety state, active TCP and payload. If a move or acquisition fails, Dashboard `stop` is sent and the run is marked interrupted. Inspect the tool and recover under pendant control; do not assume the tip was lifted after an interruption.

`manifest.json` contains the masses, nominal force, complete calibration and config. `samples.jsonl` contains one record per completed contact dwell: pad XY, commanded TCP, measured arrival and settled TCP poses, UTC timestamp, `eit_baseline` and `eit_contact`. Missing EIT observations are represented only in a motion-check run. Physical clearance, contact load, EIT acquisition and robot accuracy have not been validated by offline code checks.
