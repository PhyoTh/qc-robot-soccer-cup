# Robot Soccer Cup Drivers and Camera Setup

This repository contains the hardware-control software for the Hiwonder miniAuto robot used in the Robot Soccer Cup hosted by the Qualcomm Developer Relations Team.

No machine-learning models or inference application files are included but robot controls and camera stream work regardless of an ML model by design.

For questions or concerns, please create GitHub Issues.

## Contents

- [Robot Soccer Cup Drivers and Camera Setup](#robot-soccer-cup-drivers-and-camera-setup)
  - [Contents](#contents)
  - [System overview](#system-overview)
  - [Repository layout](#repository-layout)
  - [Using the `uno-q-miniauto` agent skill](#using-the-uno-q-miniauto-agent-skill)
    - [What the skill can build](#what-the-skill-can-build)
    - [How to use it](#how-to-use-it)
    - [Safe validation workflow](#safe-validation-workflow)
  - [Hardware map](#hardware-map)
    - [Pin Mapping](#pin-mapping)
  - [UNO Q robot setup](#uno-q-robot-setup)
    - [Requirements](#requirements)
    - [Run the example](#run-the-example)
  - [Python driver API](#python-driver-api)
  - [Sensor and health payloads](#sensor-and-health-payloads)
  - [Serial command reference](#serial-command-reference)
    - [Single-key commands](#single-key-commands)
    - [Line commands](#line-commands)
    - [Motor diagnostics](#motor-diagnostics)
  - [Hiwonder protocol compatibility](#hiwonder-protocol-compatibility)
  - [Camera setup and flashing](#camera-setup-and-flashing)
    - [Optional vendor tools](#optional-vendor-tools)
    - [One-time Arduino IDE setup](#one-time-arduino-ide-setup)
    - [Configure and flash a camera](#configure-and-flash-a-camera)
    - [Test the stream](#test-the-stream)
    - [Camera configuration](#camera-configuration)
  - [Model Import](#model-import)
  - [Safety and troubleshooting](#safety-and-troubleshooting)
    - [Robot safety behavior](#robot-safety-behavior)
    - [Robot does not respond to Python](#robot-does-not-respond-to-python)
    - [Camera initialization fails](#camera-initialization-fails)
    - [Camera serial port is missing](#camera-serial-port-is-missing)
    - [Stream is slow or choppy](#stream-is-slow-or-choppy)

## System overview

The project has two independently flashed controllers:

1. The **Arduino UNO Q** runs `sketch/sketch.ino`. It controls the motors, RGB lights, buzzer, servo/gripper, ultrasonic sensor, line sensor, battery reading, serial interface, and Router Bridge RPC interface.
2. The **Hiwonder ESP32S3-CAM V1.0** runs `camera/HiwonderCamStream.ino`. It creates a Wi-Fi access point and serves the GC2145 camera as an MJPEG stream.

The Linux side of the UNO Q runs `python/main.py`, which uses `python/robot_client.py` to call the sketch through `Arduino_RouterBridge`.

## Repository layout

```text
.
├── .agents/
│   └── skills/
│       └── uno-q-miniauto/          # Repo-local agent workflow for extending this robot
├── app.yaml                         # UNO Q App Lab application metadata
├── camera/
│   └── HiwonderCamStream.ino        # ESP32-S3 GC2145 camera firmware
├── python/
│   ├── main.py                      # Example robot motion sequence
│   └── robot_client.py              # Python wrapper for Bridge RPC methods
└── sketch/
    ├── sketch.ino                   # Complete UNO Q miniAuto hardware driver
    └── sketch.yaml                  # Zephyr platform and Bridge dependency
```

## Using the `uno-q-miniauto` agent skill

This repository includes a reusable agent skill at `.agents/skills/uno-q-miniauto`. Your AI agent can use it to turn a robot idea into focused changes across the existing UNO Q firmware, Python Bridge application, camera firmware, configuration, and documentation while preserving the driver's safety behavior and public interfaces.

### What the skill can build

| Goal | Typical result |
| --- | --- |
| Create autonomous or soccer behavior | Add sensor-driven routines, navigation policies, team behavior, or match logic on the UNO Q Linux/Python side. |
| Integrate vision or an ML model | Connect an externally supplied model to camera frames and translate detections into bounded robot actions. Model binaries remain outside this driver repository. |
| Add a sensor or actuator | Add bounded MCU I/O, safe failure handling, sensor payload fields, actuator commands, and Python wrapper methods. |
| Extend the Bridge API | Add matching `Bridge.provide_safe(...)` firmware providers and typed `MiniAutoRobot` methods without breaking the existing contract. |
| Tune or add motion | Adjust motor mapping, polarity, mecanum mixing, named movements, speed limits, and timed-stop behavior. |
| Customize the camera | Change the ESP32-S3 access point, MJPEG stream, button behavior, or GC2145 configuration. |
| Adapt a hardware revision | Update verified pins, I2C addresses, wiring documentation, diagnostics, and initialization for a different miniAuto build. |
| Improve setup and distribution | Update App Lab metadata, Arduino dependencies, public API documentation, examples, and first-run instructions. |

The skill extends the checked-out source rather than generating a separate driver stack. It chooses the smallest appropriate layer: high-level behavior normally stays in Python, direct hardware control and fail-safe timing stay in the UNO Q sketch, and camera-only changes stay in the ESP32-S3 firmware.

### How to use it

Open this repository in your platform of choice, for example `Codex`, then explicitly invoke the skill with `$uno-q-miniauto` and describe the outcome you want. Include known hardware details, constraints, and desired stop behavior when they matter.

Example prompts:

```text
Use $uno-q-miniauto to make the robot follow a line until the ultrasonic
sensor sees an obstacle within 30 cm, then stop safely.
```

```text
Use $uno-q-miniauto to add a ball-contact sensor on a verified free pin,
expose its state to Python, and document the wiring and payload field.
```

```text
Use $uno-q-miniauto to consume my externally supplied object-detection
model, track a soccer ball from the camera stream, and stop on stale frames.
```

```text
Use $uno-q-miniauto to add a timed kick actuator. Preserve the existing
Bridge API and make program disable, sensor failure, and exceptions stop it.
```

A useful request states:

- the behavior the robot should exhibit;
- which sensors, actuators, camera, or model are involved;
- verified pins, addresses, voltage levels, or wiring when adding hardware;
- timing, speed, range, or accuracy constraints; and
- what must make the robot stop or fail safely.

The agent will inspect the live repository, preserve unrelated changes, map the request to the correct layer, update all affected Bridge contract points together, and report which software and physical behaviors were actually verified.

### Safe validation workflow

The skill starts with checks that do not operate hardware:

```bash
python3 -m py_compile python/*.py
python3 .agents/skills/uno-q-miniauto/scripts/check_robot_contract.py --strict
```

The bundled contract checker verifies that the baseline firmware providers still exist, every Python Bridge call has a matching provider, the Python source parses, and the UNO Q Zephyr/Router Bridge configuration remains present.

After static checks, compile only the affected firmware with its required toolchain. Hardware tests should begin with the wheels raised, low power or speed, short timed actions, and a reachable physical disconnect. Test one motor or actuator at a time before combined or autonomous motion. The skill does not treat static analysis or successful compilation as proof that physical behavior works, and it will not flash or move a live robot unless hardware execution is explicitly requested and a safe setup is confirmed.

## Hardware map

The robot sketch intentionally targets an Arduino UNO Q using the `arduino:zephyr` platform.

### Pin Mapping

| Part | Pin or address |
| --- | --- |
| Motor PWM channels M0, M1, M2, M3 | D10, D9, D6, D11 |
| Motor direction channels M0, M1, M2, M3 | D12, D8, D7, D13 |
| Onboard WS2812 RGB data | D2 |
| Passive buzzer | D3 |
| Servo/gripper | D5 |
| Battery divider | A3 |
| Glowing ultrasonic sensor | I2C `0x77` |
| Four-channel line sensor | I2C `0x78` |

## UNO Q robot setup

### Requirements

- Arduino UNO Q
- [Pre-assembled] Hiwonder miniAuto chassis and its connected motor/sensor board
- Arduino App Lab

No third-party Python packages are required by the imported driver. The Python files use the `arduino.app_utils` runtime supplied by the UNO Q application environment.

### Run the example

Import or open this repository as an UNO Q application, build and upload `sketch/sketch.ino`, and start the application described by `app.yaml`. The Python example:

1. prints the driver health and sensor payloads;
2. stops the robot;
3. drives forward, left, right, and backward once; and
4. stops and remains idle.

The sequence can be tuned with environment variables:

| Variable | Default | Meaning |
| --- | ---: | --- |
| `ROBOCUP_SPEED` | `150` | Requested motion speed on the driver's `0..255` input scale |
| `ROBOCUP_PULSE_MS` | `700` | Duration of each motion in milliseconds |
| `ROBOCUP_PAUSE_SEC` | `0.25` | Pause before and between movements in seconds |

Keep the robot's wheels clear of the ground during the first test. The example begins moving after the application starts.

## Python driver API

`python/robot_client.py` exposes `MiniAutoRobot`, a small typed wrapper around the Bridge RPC methods registered by the Arduino sketch.

```python
from robot_client import MiniAutoRobot

robot = MiniAutoRobot()
print(robot.health())
robot.drive("forward", speed=150, ms=700)
robot.stop()
print(robot.read_sensors())
```

| Method | Behavior |
| --- | --- |
| `drive(command, speed=150, ms=500)` | Drives in a named direction. Speed is reduced to an absolute `0..255` value; duration is clamped to `0..5000` ms. A duration of `0` continues until stopped. |
| `stop()` | Stops all motors and disables obstacle avoidance. |
| `read_sensors()` | Returns the parsed JSON sensor payload as a dictionary, or `{}` if the Bridge returns no payload. |
| `servo(angle)` | Moves the servo/gripper; the sketch clamps the angle to `0..180` degrees. |
| `buzz()` | Plays a three-part buzzer chirp. |
| `led(on)` | Sets the onboard RGB and ultrasonic RGB lights to white or off. |
| `drive_raw(m0, m1, m2, m3, ms=500)` | Sets individual motor inputs, each clamped to `-255..255`, with an optional auto-stop duration. |
| `health()` | Returns the parsed driver identity and interface status. |

Accepted `drive` directions and aliases:

| Motion | Accepted values |
| --- | --- |
| Forward | `forward`, `f` |
| Backward | `backward`, `back`, `reverse`, `b` |
| Strafe left | `left`, `strafe_left`, `a` |
| Strafe right | `right`, `strafe_right`, `d` |
| Rotate left | `rotate_left`, `turn_left`, `q` |
| Rotate right | `rotate_right`, `turn_right`, `e` |
| Stop | `stop`, `x` |

An unrecognized direction returns `False` without starting a new movement.

## Sensor and health payloads

A successful `read_sensors()` call returns this shape:

```json
{
  "robot": "hiwonder_miniauto",
  "mcu": "uno_q",
  "ir": -1,
  "line_ok": true,
  "line_digital": [0, 1, 1, 0],
  "trace_digital": [0, 1, 1, 0],
  "ultrasonic_mm": 250,
  "ultrasonic_cm": 25,
  "battery_mv": 7400
}
```

- `line_digital` and `trace_digital` intentionally contain the same four bits for caller compatibility.
- `line_ok` is `false` if the line-sensor I2C read fails; its bits are then zeroed.
- `ultrasonic_mm` and `ultrasonic_cm` are `-1` if the ultrasonic I2C read fails.
- `ir` is a reserved compatibility field and is always `-1` in this driver.
- `battery_mv` is derived from the A3 reading using the driver's miniAuto divider calibration factor.

The health payload is:

```json
{
  "robot": "hiwonder_miniauto",
  "mcu": "uno_q",
  "bridge": true,
  "serial": true
}
```

## Serial command reference

Open the UNO Q monitor at `9600` baud. The sketch accepts newline-terminated commands, single-key commands, Hiwonder pipe commands ending in `&`, and parenthesized commands. Buffered input is also processed after an 80 ms idle timeout.

### Single-key commands

| Key | Action |
| --- | --- |
| `?` | Print command help. |
| `r` | Print sensor JSON. |
| `l` | Blink the onboard and ultrasonic RGB lights. |
| `z` | Play a buzzer chirp. |
| `u` | Print ultrasonic distance in millimeters. |
| `v` | Run servo center-open-close-center test. |
| `1`, `2`, `3`, `4` | Pulse one motor channel forward and backward for hardware mapping. |
| `f`, `b`, `a`, `d` | Drive forward, backward, left, or right using default settings. |
| `q`, `e` | Rotate left or right using default settings. |
| `x` | Stop all motors and disable obstacle avoidance. |

Single-key motion commands use speed `180` and a `700` ms pulse.

### Line commands

Separators `,`, `|`, `(`, `)`, and `&` are normalized, so both `servo 90` and `servo(90)` are accepted where applicable.

| Command | Example | Notes |
| --- | --- | --- |
| `drive <direction> [speed] [ms]` | `drive forward 180 700` | Defaults to speed `180`; omitted duration is `0` (continuous). |
| `stop` | `stop` | Stops motors and obstacle avoidance. |
| `read_sensors` or `sensors` | `read_sensors` | Prints sensor JSON. |
| `servo <angle>` | `servo 90` | Clamped to `0..180`. |
| `buzz` | `buzz` | Plays a chirp. |
| `led <0/1>` | `led 1` | Sets both RGB light paths to white or off. |
| `rgb <r> <g> <b>` | `rgb 255 0 0` | Each channel is clamped to `0..255`. |
| `drive_raw <m0> <m1> <m2> <m3> [ms]` | `drive_raw 120 120 120 120 500` | Each motor is clamped to `-255..255`; omitted duration is continuous. |
| `health` | `health` | Prints health JSON. |

### Motor diagnostics

These commands physically move the robot. Lift the wheels and keep hands, cables, and tools clear.

| Command | Purpose |
| --- | --- |
| `dir_sweep <0..3>` | Tests the known direction-pin candidates for one sketch motor. |
| `dir_scan <0..3>` | Scans the broader non-PWM header-pin set for a working direction signal. |
| `combo_scan` | Tries PWM-only channel combinations as fallback movement tests. |
| `pwm_combo <mask> <m2dir> <ms> <speed>` | Runs one PWM-only combination. Mask bits are `1=M0`, `2=M1`, `4=M2`, and `8=M3`. |

## Hiwonder protocol compatibility

The sketch accepts Hiwonder-style pipe-delimited commands ending in `&`.

| Command | Action |
| --- | --- |
| `A\|state\|&` | Select a motion state. |
| `B\|r\|g\|b\|&` | Set both RGB light paths. |
| `C\|speed\|&` | Set speed percent, clamped to `10..100`. |
| `D\|&` | Print `$ultrasonic_mm,battery_mv$`. |
| `E\|increase\|&` | Move the servo to `90 + increase`; increase is clamped to `0..60`. |
| `F\|0\|&` or `F\|1\|&` | Disable or enable obstacle avoidance. Disabling also stops the motors. |

Motion states for `A`:

| State | Motion |
| ---: | --- |
| `0` | Left |
| `1` | Forward-left |
| `2` | Forward |
| `3` | Forward-right |
| `4` | Right |
| `5` | Backward-right |
| `6` | Backward |
| `7` | Backward-left |
| `8` | Stop |
| `9` | Rotate left |
| `10` | Rotate right |
| `11` | Stop |

Any other motion state also stops the robot. When obstacle avoidance is enabled, the sketch checks every 100 ms: it rotates left below 400 mm and otherwise drives forward at the selected speed.

## Camera setup and flashing

`camera/HiwonderCamStream.ino` is the firmware for the **Hiwonder ESP32S3-CAM V1.0 with GC2145 sensor**. It replaces the stock camera image with an MJPEG server and gives each robot its own Wi-Fi SSID.

### Optional vendor tools

The source documentation references these externally hosted files:

- [CH341/CH34x camera serial driver](https://drive.google.com/drive/folders/1CJBYFEaHWPLZ6eSSgGjHhziZFMqmF-mv)
- [Camera flash erase tool](https://drive.google.com/drive/folders/1iDdatjYswiquF1eNqKYVFBq68VrKZV_U)
- [Original `image_transmit.bin`](https://drive.google.com/drive/folders/1YOCjBNvqUxpelmbY5Be6dNHE6siZCAke)

These are third-party Google Drive links inherited from the preparation repository. Review downloaded software according to your organization's security policy before installing or running it.

### One-time Arduino IDE setup

1. Open Arduino IDE and go to **Tools → Board → Boards Manager**.
2. Search for `esp32`.
3. Install **esp32 by Espressif Systems**, version **2.0.11**.

The preparation notes require 2.0.11 because ESP32 core 3.x was observed to fail GC2145 initialization with this firmware.

Select these options before each upload:

| Setting | Value |
| --- | --- |
| Board | `ESP32S3 Dev Module` |
| PSRAM | `OPI PSRAM` |
| Flash Size | `8MB (64Mb)` |
| Partition Scheme | `Huge APP (3MB No OTA/1MB SPIFFS)` |
| USB CDC On Boot | `Disabled` |
| Flash Mode | `DIO` |
| Upload Speed | `921600` |
| Port | The serial/COM port assigned to the camera |

### Configure and flash a camera

1. Open `camera/HiwonderCamStream.ino` in Arduino IDE.
2. Set a unique `CAMERA_SSID` for the robot. The checked-in default is `miniAuto_CAM_01`.
3. Set `CAMERA_PASS` if needed. The checked-in password is `hiwonder123`; it satisfies the ESP32 soft-AP minimum of eight characters.
4. Click **Upload**.
5. Confirm the output ends with a verified hash and hard reset, then power-cycle the camera.

For a fleet, change the SSID to `miniAuto_CAM_02`, `miniAuto_CAM_03`, and so on before flashing each additional camera. The IP address and stream URL remain the same because a client connects to only one camera access point at a time.

### Test the stream

1. Connect a phone or computer to the camera's configured SSID using its configured password.
2. Open `http://192.168.5.1:81/`.
3. Select **Start Stream**.

The direct MJPEG endpoint is:

```text
http://192.168.5.1:81/stream
```

A Python or OpenCV consumer should use that URL after the host has joined the camera's Wi-Fi access point:

```python
CAMERA_URL = "http://192.168.5.1:81/stream"
```

### Camera configuration

The firmware starts a soft access point at `192.168.5.1/24`, listens on port `81`, captures QVGA frames (`320×240`) in the GC2145's native RGB565 format, converts them to JPEG at quality `80`, and serves a multipart MJPEG response. Two frame buffers are allocated in PSRAM.

Camera pin definitions:

| Signal | GPIO | Signal | GPIO |
| --- | ---: | --- | ---: |
| PWDN | `-1` | RESET | `-1` |
| XCLK | `15` | PCLK | `13` |
| SIOD/SDA | `4` | SIOC/SCL | `5` |
| Y2 | `11` | Y3 | `9` |
| Y4 | `8` | Y5 | `10` |
| Y6 | `12` | Y7 | `18` |
| Y8 | `17` | Y9 | `16` |
| VSYNC | `6` | HREF | `7` |

The XCLK frequency is `15 MHz`. These values come from Hiwonder's `camera_setting.h` for the `CAMERA_MODEL_ESP32S3_EYE` mapping used by the source firmware.

## Model Import

[TK]

## Safety and troubleshooting

### Robot safety behavior

- Named and raw drive durations are clamped to `0..5000` ms.
- A nonzero duration arms an automatic stop timer; `0` means continuous motion.
- Named speed inputs and raw motor inputs are clamped before PWM output.
- `stop()`, serial `stop`, and serial `x` stop every motor and disable obstacle avoidance.
- The obstacle-avoidance loop turns when a valid reading is below `400` mm. A failed or nonpositive distance reading follows the forward branch, so supervise this demo behavior closely.
- The battery value is an estimate based on the hard-coded divider calibration, not a precision fuel gauge.

Always begin with wheels raised, use short timed commands, and keep a physical power disconnect within reach.

### Robot does not respond to Python

- Confirm the UNO Q sketch built with the `arduino:zephyr` profile.
- Confirm `Arduino_RouterBridge (0.4.2)` was installed from `sketch/sketch.yaml`.
- Run `health` in the serial monitor. The response should report both `serial` and `bridge` as `true`.
- Run the single-motor tests with the wheels raised to verify the physical mapping.

### Camera initialization fails

- Confirm the board package is Espressif ESP32 version `2.0.11`, not 3.x.
- Confirm the board, PSRAM, flash, partition, USB CDC, and flash-mode settings match the table above.
- Upload again, then fully power-cycle the camera.

### Camera serial port is missing

- Try a known data-capable USB cable and another USB port.
- On Windows, install the appropriate CH34x driver if Device Manager shows an unknown USB serial device.
- Reopen Arduino IDE after the driver is installed.

### Stream is slow or choppy

This can be expected: the GC2145 supplies RGB565 frames and the ESP32-S3 converts every frame to JPEG in software. Stay close to the camera access point, disconnect unused clients, and keep the default QVGA frame size while diagnosing performance.
