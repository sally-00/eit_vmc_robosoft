# ESP32 joystick readout

This records raw joystick deflection for the planned physical joystick experiment. It does not move the robot. The serial output includes the ESP32 millisecond clock; the computer saves its own wall and monotonic timestamps alongside each sample for alignment with robot and EIT logs.

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

On this computer, fully quit any running Arduino IDE windows, then launch the downloaded IDE with serial-port access:

```bash
sg dialout -c "$HOME/Downloads/arduino-ide_2.3.10_Linux_64bit.AppImage"
```

The account is already in `dialout`, but existing applications may still have the old group membership. This command applies the group when launching Arduino IDE. A full computer restart should also refresh group membership for applications.

1. Install Arduino IDE. In **File > Preferences > Additional Boards Manager URLs**, add `https://espressif.github.io/arduino-esp32/package_esp32_index.json`. In **Tools > Board > Boards Manager**, search for `esp32` and install **esp32 by Espressif Systems**. You can skip this installation if ESP32 board support is already present.
2. Connect the Freenove ESP32-WROVER board to this computer with a USB **data** cable. Open `esp32_joystick/esp32_joystick.ino` in Arduino IDE.
3. Select **Tools > Board > ESP32 Arduino > ESP32 Wrover Module**. Under **Tools > Port**, select the port that appeared when you connected the board, usually `/dev/ttyUSB0` or `/dev/ttyACM0` on Linux.
4. Click **Upload** (the right-arrow button) and wait for the success message. If the uploader stalls at `Connecting...`, try holding the board's **BOOT** button until uploading starts, then release it.
5. To check the readings, open **Tools > Serial Monitor**, set **115200 baud**, and move the stick. You should see the CSV header and changing numbers. Close Serial Monitor before starting the Python reader because both programs cannot hold the same port at once.

The firmware sends `esp_ms,x_raw,y_raw,pressed` CSV at nominally 100 samples/s and 115200 baud. `x_raw` and `y_raw` are 12-bit ADC counts (0 to 4095). `pressed` is 1 while the stick button is held.

## Plot and save on the computer

Miniconda is installed at `/home/birl/Documents/miniconda3`, with `pyserial` and `matplotlib` already installed in its base environment. Close Arduino Serial Monitor, then run:

```bash
sg dialout -c "$HOME/Documents/joystick_hardware/run_joystick.sh --port /dev/ttyUSB0"
```

The launcher uses the installed Miniconda interpreter, ignores inherited Python environment variables from other software, and saves each run to a new CSV file in `~/Documents/joystick_hardware/recordings/`. The live window plots both analog channels; close it or press Ctrl+C to stop.

For recording without a plot:

```bash
sg dialout -c "$HOME/Documents/joystick_hardware/run_joystick.sh --port /dev/ttyUSB0 --no-plot"
```

If the USB device has another name, substitute the port shown in Arduino IDE or `ls /dev/ttyUSB* /dev/ttyACM*`. To choose a file, add `--output recordings/my_trial.csv` inside the quoted command. Existing files are never overwritten.

The account `birl` is already in `dialout`. If configuring a different account, add it once with `sudo usermod -aG dialout USERNAME`, replacing `USERNAME` with that account's name, then restart the computer. After group membership takes effect, the launcher can also be run directly:

```bash
cd ~/Documents/joystick_hardware
./run_joystick.sh --port /dev/ttyUSB0
```

To activate Miniconda manually in a terminal:

```bash
source ~/Documents/miniconda3/etc/profile.d/conda.sh
conda activate base
```

For the paper, these raw counts must be calibrated to physical joystick angle using measured centre and known positive/negative angles. Confirm which analog axis follows the planned one-axis motion and record its direction. The joystick reading measures angle for evaluation; it does not measure contact force. The ESP32 and PC timestamps have different clock origins, and USB serial arrival adds delay, so align the streams before computing tracking error.
