# ESP32 joystick readout

The ESP32 is connected by USB to the same computer that runs the robot experiment. The uploaded sketch streams raw joystick readings. `franka_eit_vmc`'s `run_block.py` starts `run_joystick.sh` automatically for joystick blocks and saves one recording in the block folder. Use `run_joystick.sh` directly only for standalone sensor checks; do not run both readers on the same port. The sketch alone does not create a PC recording or retain a completed trial. The recorder does not move the robot. The serial output includes the ESP32 millisecond clock; the computer saves its own wall and monotonic timestamps alongside each sample for alignment with robot and EIT logs.

The sketch uses the wiring you provided for the KY-023 / PS2 module:

| Joystick pin | ESP32-WROVER connection |
| --- | --- |
| `+5V` or `VCC` | **3V3** (despite any `+5V` label; the ESP32 analog input must stay at or below 3.3 V) |
| `GND` | GND |
| `VRx` | GPIO32 (ADC1) |
| `VRy` | GPIO33 (ADC1) |
| `SW` | GPIO14 (pull-up is enabled in the sketch) |

GPIO32 and GPIO33 are ADC1 pins. GPIO14 reads the button as a digital input with an internal pull-up. Power the module from 3V3, as wired, so the joystick outputs remain within ESP32 input range.

## Upload to the ESP32

The `.ino` sketch must be uploaded once to the ESP32, and uploaded again after any changes to its code. It then starts automatically whenever the board is powered.

On this computer, fully quit any running Arduino IDE windows, then launch the downloaded IDE:

```bash
$HOME/Downloads/arduino-ide_2.3.10_Linux_64bit.AppImage
```

Serial access requires the user account to belong to `dialout`; the instructions below cover a permission error.

1. Install Arduino IDE. In **File > Preferences > Additional Boards Manager URLs**, add `https://espressif.github.io/arduino-esp32/package_esp32_index.json`. In **Tools > Board > Boards Manager**, search for `esp32` and install **esp32 by Espressif Systems**. You can skip this installation if ESP32 board support is already present.
2. Connect the Freenove ESP32-WROVER board to this computer with a USB **data** cable. Open `esp32_joystick/esp32_joystick.ino` in Arduino IDE.
3. Select **Tools > Board > ESP32 Arduino > ESP32 Wrover Module**. Under **Tools > Port**, select the port that appeared when you connected the board, usually `/dev/ttyUSB0` or `/dev/ttyACM0` on Linux.
4. Click **Upload** (the right-arrow button) and wait for the success message. If the uploader stalls at `Connecting...`, try holding the board's **BOOT** button until uploading starts, then release it.
5. To check the readings, open **Tools > Serial Monitor**, set **115200 baud**, and move the stick. You should see the CSV header and changing numbers. Close Serial Monitor before starting the Python reader because both programs cannot hold the same port at once.

The firmware sends `esp_ms,x_raw,y_raw,pressed` CSV at nominally 100 samples/s and 115200 baud. `x_raw` and `y_raw` are 12-bit ADC counts (0 to 4095). `pressed` is 1 while the stick button is held.

## Plot and save on the computer

First find the ESP32's actual port. `/dev/ttyUSB0` is an example; an `[Errno 2] No such file or directory` error means the named port does not exist, not that it needs permission. Connect the board with a USB **data** cable and run:

```bash
/usr/bin/python3 -m serial.tools.list_ports -v
ls -l /dev/serial/by-id/  # if present, these names remain stable across reconnects
```

Use the detected `/dev/ttyUSB*`, `/dev/ttyACM*`, or `/dev/serial/by-id/*` path in `--port`. If no serial port appears, check the cable, USB connection, and whether the board appears in `lsusb` before changing permissions.

If opening an existing port instead says `Permission denied`, grant this account access to serial devices and **log out and back in** before retrying:

```bash
sudo usermod -aG dialout "$USER"
```

In the new login session, verify access and use the actual device name:

```bash
id -nG  # must include dialout
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

The same `dialout` membership covers the ESP32 and EIT device when their serial nodes are group-owned by `dialout`. The launcher uses system Python 3.10 with `pyserial` and `matplotlib`. Close Arduino Serial Monitor, then run from this checkout:

```bash
./joystick_hardware/run_joystick.sh --port /dev/ttyUSB0
```

The launcher ignores inherited Python environment variables from other software and saves each run to a new CSV file in `joystick_hardware/recordings/`. The live window plots both analog channels; close it or press Ctrl+C to stop.

For recording without a plot:

```bash
./joystick_hardware/run_joystick.sh --port /dev/ttyUSB0 --no-plot
```

If the USB device has another name, substitute the port shown in Arduino IDE or `ls /dev/ttyUSB* /dev/ttyACM*`. To choose a file, add `--output recordings/my_trial.csv` to the launcher command. Existing files are never overwritten.

For the paper, raw counts are converted to approximate physical joystick angle using the measured centre and the four ±35° endpoints in `../franka_eit_vmc/config/joystick_adc_calibration.json` (recorded with the joystick supplied from 3V3). The 0/4095 ADC rails make near-stop angles approximate; validate intermediate angles independently if accurate error measurements are required. Confirm which analog axis follows the planned one-axis motion and record its direction. The joystick reading measures angle for evaluation; it does not measure contact force. The ESP32 and PC timestamps have different clock origins, and USB serial arrival adds delay, so align the streams before computing tracking error.
