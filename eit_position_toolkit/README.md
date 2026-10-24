# EIT position training and testing

This folder owns the standalone position-only workflow extracted from the UR3e collection tools. It collects manual pad touches, trains an RBF or NumPy MLP model, and tests contact-position predictions. It has no ROS, robot-control or force-label dependency. The serial reader and both numbered-plate view meshes are local. The weighted UR3e collector remains in `../ur3e_weight_position_data_collection/ur3e_code/`. Its former position-only sessions and models have moved here to `contact_position_runs/`, with their folder names unchanged; historical signal reports and preview images are in `signal_diagnostics/`.

## Start here

1. Open a terminal in this folder and install dependencies:

   ```bash
   python3 -m pip install -r requirements.txt
   ```

   Tkinter must also be available (`python3-tk` on Ubuntu). Close any other program reading this EIT device.

2. Check `config.json`: `eit_port`, the actual electrode pins in physical order, drive pattern and frequency. The copied settings are 16 electrodes, combined drive, 50 kHz, two extra RX gain halvings and **allow clipped readings enabled**, matching the existing setup. The 1–8 engraved landmarks are independent of the 8/16 electrode pins. Do not infer wiring from the landmark numbers.

3. Power/connect the device using its documented external supply and USB-HS connection. If opening the port reports **Permission denied**, follow [USB port permissions](#usb-port-permissions) below. Leave the pad untouched, then launch:

   ```bash
   python3 contact_position_app.py --points 20 --repeats 3 --pins 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 --port /dev/ttyACM0
   ```

4. Select **Connect (pad untouched)**. Inspect the signal plots and **Channel diagnostics** tab before collecting. Look for persistent zeros, clipping, unstable untouched readings and channels close to their estimated ceiling. Use **Save signal snapshot…** to preserve raw frames, baseline, active gains and diagnostics for investigation.

5. Select **New training**. For each guided point, lift off and **Capture untouched baseline**, then touch/hold the orange cross and **Capture held touch**. Complete all points and repetitions. Only these capture buttons save training touches; live previews do not advance the plan.

6. Select **Train model**. The session contains `touches.jsonl`, `manifest.json`, the model, its evaluation and `signal_quality.json`. The last file lists clipping, zero channels, baseline variation, weak touch changes and peak estimated range usage per touch. These are advisories; they do not silently remove or change samples. The model's existing zero-baseline channel exclusion and Allow clipped setting still apply. You can select the other model type and fit the same collected touches again.

7. Select **Live test**, release the pad and capture a new untouched test baseline. Poke known positions, including positions between training points. Compare the estimated dot with the actual position. Strength/familiarity readouts are diagnostics, not accuracy guarantees. As in the original app, weak/unfamiliar signals still return predictions; a dot after release is not proof of contact.

8. Copy selected model files to `models/` to share them with another computer. Future sessions are saved locally in `contact_position_runs/` using Japan timestamps. Raw sessions and snapshots are ignored by Git; models placed in `models/` and code/view assets are tracked.

To test an existing model, use **Load model…**, including models migrated from the old UR3e sessions into this folder, or:

```bash
python3 contact_position_app.py --model /path/to/position_model.npz
```

The model restores its saved scan settings. Saved sessions are not automatically resumed. To review a session again without a sensor:

```bash
python3 review_session.py --session /path/to/session
```

To exercise the GUI with synthetic signals and no EIT hardware:

```bash
python3 contact_position_app.py --demo --points 5 --repeats 2
```

## USB port permissions

On Ubuntu, the EIT USB serial device normally belongs to the `dialout` group. The currently identified device is **`/dev/ttyACM0`**, with the persistent path `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_376D395E3033-if00` in `config.json`.

For **immediate access from your current desktop/IDE session**, run this in your terminal and enter your Linux password when prompted:

```bash
sudo setfacl -m u:"$USER":rw /dev/ttyACM0
```

Then retry **Connect (pad untouched)**. This grants your user read/write access to the actual USB serial port. It can be lost when the EIT is unplugged or re-enumerates; repeat it if necessary. If `setfacl` is unavailable, install it with `sudo apt install acl`.

Check the current device number after reconnecting:

```bash
ls -l /dev/serial/by-id/
```

Use the linked `/dev/ttyACM…` device in the permission command if it is no longer `ttyACM0`. You can also grant access through the configured persistent link:

```bash
sudo setfacl -m u:"$USER":rw /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_376D395E3033-if00
```

For **persistent group access** on another Ubuntu account/computer:

```bash
sudo usermod -aG dialout "$USER"
```

Sign out of the desktop and sign back in, then restart your IDE/trainer. The existing desktop session does not automatically acquire new group membership. On this computer, `birl` is already a member of `dialout`; you can activate it in a terminal immediately:

```bash
newgrp dialout
cd "/media/birl/Yi Cam data/09_EIT_vmc/eit_position_toolkit"
python3 contact_position_app.py --points 20 --repeats 3
```

Run the trainer from that new shell. Alternatively, from this toolkit folder, launch one process with the group enabled:

```bash
sg dialout -c 'python3 contact_position_app.py --points 20 --repeats 3'
```

## What the voltage diagnostics mean

The firmware's `phase` stream reports the **fundamental RMS voltage before RX amplification**, with a phase and a separate `C` clipping flag. The plot shows those reported amplitudes, averaged over the capture. It is not an oscilloscope waveform or the instantaneous ADC voltage.

A zero reading means the firmware reported zero fundamental amplitude. It does not by itself identify an open electrode, a broken wire or a healthy small signal. A clipped reading means ADC rail hits were detected; the reported amplitude is unreliable even if numerically small. Severe saturation can produce an almost constant waveform with little fundamental component, while gain correction can also make the reported number small. These are possible explanations, not a diagnosis of a specific zero channel. The software preserves both the raw values and flags; it does not replace clipped readings with zero. A channel can have both flags.

The supplied v1.1.1 firmware converts the 12-bit ADC to −2048…2047 at 1 mV/count, calculates the fundamental RMS amplitude and divides by the active RX gain. It sets `C` when more than one acquired sample hits an ADC rail. For a centred sinusoid, the estimated pre-RX ceiling is therefore:

```
estimated input RMS ceiling = 2.047 V / (sqrt(2) * active RX gain)
estimated range use (%) = 100 * maximum reported RMS in capture / ceiling
```

For example, RX ×1 gives approximately **1447 mV RMS**, ×8 gives **181 mV RMS**, ×32 gives **45.2 mV RMS**, and ×256 gives **5.65 mV RMS**. There is no single mV limit for the entire scan: autogain sets gains per configuration. The dashed ceiling in the voltage plot and the range-usage plot use the **active gain dump**, including extra RX backoff. The table shows the actual TX/RX electrode pairs, gain, mean voltage, estimated ceiling, peak clean usage, clipping/zero frame percentages and within-capture variation.

**Range usage is an estimate, not measured ADC peak headroom.** A distorted waveform, DC offset or saturation in an earlier analogue stage can invalidate the sinusoidal assumption. Firmware v1.2.1 was reported by the existing setup; only v1.1.1 source is supplied, so that source does not prove identical analogue scaling in the connected version. `eit_adc_peak_limit_v` can be changed if a verified hardware/firmware calibration gives a different value. Without a valid gain dump, or in demo mode, the range is shown as unknown. Clipped channels never receive a trustworthy usage percentage: their crosses appear above 100% as a status marker, not an amplitude measurement.

The 80% line is an advisory to leave room for touch-induced changes; it is not a manufacturer guarantee. The 5% low-use advisory is also only a tuning aid. Do not try to fill the ADC range on every channel at the expense of clipping others. Compare untouched and representative held touches. The frontend's noise floor and waveform are not streamed, so the plot cannot establish a universal minimum detectable voltage or effective resolution. Within-capture variation during movement includes real motion and is not pure electronic noise.

If clipping occurs, release the pad, stop/restart the app and increase **Extra RX gain halvings after autogain** before connecting. One extra step halves RX gain and doubles the estimated input ceiling; the reported unclipped pre-RX voltage should remain roughly similar. Gain is not changed during collection. Capture fresh baselines and collect new training data after changing gain. Allow clipped readings remains available, but accepting a reading cannot repair its distortion.

## Model export to the Franka package

Training stays here; the ROS package only predicts. Its current decoder accepts an RBF model with all **40 adjacent-scan channels from eight electrodes**. This toolkit also supports 16-electrode/combined scans and MLPs; those are not currently compatible with that ROS decoder. A verified compatible checkpoint can be exported here:

```bash
python3 export_franka_model.py --input /path/to/trained/position_model.npz \
  --output models/franka_position_model.npz
```

The exporter preserves the input checkpoint. Copy the exported file to the robot computer and follow `franka_eit_vmc/README.md`.

## Verification and source

```bash
python3 -m unittest discover -p 'test_*.py'
```

Tests are synthetic/offline, not pad-accuracy evidence. GUI workflow tests require a display. Model tests cover both scan sizes and model types. Signal-range tests cover gain conversion, invalid metadata, low reported clipped values, zeros and snapshot serialization. The local meshes preserve the existing engraved-face convention: CCW 1 bottom, 3 right, 5 top, 7 left.

The range calculation was checked against the supplied `Amodo Files/Firmware/ARIA_EITSYS_FW-v1.1.1.zip`, `CM7/App/app.c`: `ADC12020_rawToSignedCode`, `onBufferReady`, `doImpedanceMeasurement`, and the streaming RX-gain correction. No device was connected or reconfigured during this extraction.
