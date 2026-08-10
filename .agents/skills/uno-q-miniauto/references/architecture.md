# UNO Q miniAuto architecture

Read this reference before modifying the robot. Re-check the live files because the repository may evolve after this skill was written.

## System boundaries

- `sketch/sketch.ino`: Arduino UNO Q MCU firmware. Own direct hardware I/O, mecanum mixing, command parsing, motor timeouts, start/stop state, sensors, servo, buzzer, LEDs, and Router Bridge RPC providers.
- `python/robot_client.py`: Linux-side typed facade. Own Bridge serialization, program-running guards, routine interruption, and caller-friendly return values.
- `python/main.py`: Example App Lab behavior. Use it as the starting point for custom policies and sequences, or move substantial behavior into focused modules under `python/`.
- `sketch/sketch.yaml`: UNO Q `arduino:zephyr` platform and `Arduino_RouterBridge` dependency.
- `app.yaml`: App Lab application metadata.
- `camera/HiwonderCamStream/HiwonderCamStream.ino`: Independent Hiwonder ESP32-S3 GC2145 camera firmware. It hosts an MJPEG stream and exposes its button state to the UNO Q over I2C.
- `README.md`: Software setup, API, payload, protocol, camera, and troubleshooting contract.
- `docs/MANUAL.md`: Physical assembly, wiring illustrations, charging, battery, and handling guidance.
- `miniAutoDriver.zip`: Distribution snapshot, not the editable source of truth. Modify the unpacked repository files unless the user explicitly asks to rebuild the archive.

## Current hardware map

| Device | UNO Q pin or bus address |
| --- | --- |
| Motor PWM M0..M3 | D10, D9, D6, D11 |
| Motor direction M0..M3 | D12, D8, D7, D13 |
| Onboard WS2812 RGB | D2 |
| Passive buzzer | D3 |
| Servo/gripper | D5 |
| Battery divider | A3 |
| Glowing ultrasonic sensor | I2C `0x77` |
| Four-channel line sensor | I2C `0x78` |
| Camera/button controller | I2C `0x79` |

Do not assume a different miniAuto revision shares this map. When hardware differs, require a pin/address table or a verified wiring diagram and retain a safe stop path while remapping.

## Current Bridge contract

The sketch registers these providers with `Bridge.provide_safe(...)` and `MiniAutoRobot` wraps them:

- `drive(command, speed, duration_ms)`
- `stop()`
- `read_sensors()` returning JSON
- `servo(angle)`
- `buzz()`
- `led(on)`
- `drive_raw(m0, m1, m2, m3, duration_ms)`
- `health()` returning JSON

`MiniAutoRobot.drive`, `servo`, and `drive_raw` are guarded by program state. Timed `drive` waits for completion and checks again so the CAM button can interrupt a routine. Keep this behavior for user-facing motion APIs.

The sensor JSON currently includes robot and MCU identity, IR compatibility state, line sensor status/bits, ultrasonic distance in millimeters and centimeters, battery millivolts, program-enabled state, and the camera hold toggle. Inspect `readSensorsJson()` and `README.md` for exact live field names.

## Extension patterns

### Add high-level robot behavior

1. Read state through `MiniAutoRobot`.
2. Make a bounded decision in Python.
3. Issue short timed actions rather than unbounded motion where possible.
4. Stop on completion, exception, disabled program state, invalid sensor data, or loss of the expected condition.
5. Keep the `App.run(user_loop=...)` callback responsive; split large behavior into testable functions or modules.

### Add a sensor or actuator

1. Define pins, addresses, units, ranges, and failure values near the existing hardware constants.
2. Search the entire sketch for every use of a proposed pin. Remove it from diagnostic scan arrays or other alternate functions before assigning it to hardware.
3. Implement bounded MCU access and a safe failure mode. Integrate actuator shutdown with explicit stop, program disable, startup, timeout, and relevant sensor-loss paths.
4. Add data to `readSensorsJson()` for read-mostly state, or create a focused RPC for actions/configuration.
5. Register the RPC with `Bridge.provide_safe(...)`.
6. Wrap it in `MiniAutoRobot`; use precise types and units in the method name, parameters, or docstring.
7. Add a call-site example and update the README contract.

### Add or change motion

1. Confirm physical wheel order, polarity, and mecanum orientation before changing mixing math.
2. Preserve per-channel clamping and the timer-driven stop.
3. Add a named command to `driveCommand(...)` when it is a stable public motion; use `drive_raw` only for diagnostics or carefully bounded experiments.
4. Update accepted aliases and README tables.
5. Test individual channels, then translation, then rotation, then combined motion.

### Add vision or an ML model

1. Keep camera capture/stream transport separate from the movement fail-safe.
2. Put inference and policy code on the Linux/Python side unless MCU execution is an explicit, feasible requirement.
3. Make camera endpoint, thresholds, labels, and team selection configurable rather than hard-coded across modules.
4. Define behavior for stale frames, no detections, ambiguous detections, and unavailable models; the safe default is stop.
5. Keep model artifacts outside this repository and document how the user supplies them at runtime.

## Static inspection shortcuts

```bash
rg -n 'Bridge\.provide_safe|Bridge\.call' sketch/sketch.ino python
rg -n 'MOTOR_|PIN_|I2C_ADDR|MAX_DRIVE_MS|DEFAULT_' sketch/sketch.ino
rg -n 'CAMERA_SSID|CAMERA_PASS|AP_IP|STREAM_PORT|GPIO|I2C' camera/HiwonderCamStream/HiwonderCamStream.ino
rg -n '^#{1,4} ' README.md docs/MANUAL.md
```

Use the bundled checker after any firmware, wrapper, or configuration change. It statically checks that the baseline RPCs remain present, Python calls have firmware providers, the Python files parse, and the UNO Q profile still declares the required platform and Bridge library.
