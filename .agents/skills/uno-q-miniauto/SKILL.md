---
name: uno-q-miniauto
description: Create and adapt Hiwonder miniAuto robots on Arduino UNO Q using this repository's firmware, Python Bridge client/application, App Lab configuration, and ESP32-S3 camera firmware. Use when building a custom miniAuto variant, autonomous routine, soccer behavior, sensor or actuator feature, Bridge RPC, motor/chassis tuning, camera configuration, or when validating related UNO Q changes. Do not use for unrelated robots or generic Arduino projects.
---

# Create an UNO Q miniAuto

Extend the working robot in this repository instead of generating a second driver stack. Preserve its safety behavior and firmware-to-Python contract while changing only the layer required by the requested robot variant.

## Start here

1. Find the repository root with `git rev-parse --show-toplevel` and work from it.
2. Read [references/architecture.md](references/architecture.md) completely before editing. Treat the checked-out source as authoritative if it differs from the reference.
3. Inspect `git status --short`. Preserve user changes and avoid unrelated rewrites.
4. Translate the request into observable behavior, inputs, outputs, timing, and a safe stop condition. Ask only for a choice that would materially change the design and cannot be inferred from the repository.
5. Run the baseline contract check:

   ```bash
   python3 .agents/skills/uno-q-miniauto/scripts/check_robot_contract.py
   ```

## Choose the customization layer

| Requested change | Primary files | Also change |
| --- | --- | --- |
| Autonomous sequence, soccer strategy, model integration, or sensor-driven policy | `python/main.py` or a focused new module under `python/` | `python/robot_client.py` only if the existing API is insufficient |
| New actuator, sensor, safety rule, motor behavior, or MCU capability | `sketch/sketch.ino` | `python/robot_client.py` and the Python application when exposed through Bridge |
| Bridge API addition or signature change | `sketch/sketch.ino` and `python/robot_client.py` | Call sites, payload docs, and validation |
| Camera AP, stream, button, or GC2145 behavior | `camera/HiwonderCamStream/HiwonderCamStream.ino` | Consumer configuration or docs that depend on its URL/protocol |
| App identity or Arduino dependency | `app.yaml` or `sketch/sketch.yaml` | README only when setup or public behavior changes |
| Physical assembly or wiring guidance | `docs/MANUAL.md` and its existing assets | Firmware pin mapping if the actual wiring changes |

Prefer a Python-only implementation for high-level behavior. Change firmware only when the hardware interface, real-time control, or safety boundary requires it. Keep camera firmware independent unless the requested feature explicitly couples it to the UNO Q.

## Implement safely

- Preserve `robot.stop()` cleanup and the firmware's timed auto-stop behavior.
- Keep motor, servo, duration, and sensor inputs bounded at the firmware boundary. Validate again in Python when it improves caller feedback, but never rely on Python as the only physical safety boundary.
- Keep long-running policy and inference work on the UNO Q Linux/Python side. Keep direct I/O, motor mixing, and fail-safe timing on the MCU side.
- Reuse `MiniAutoRobot` rather than scattering raw `Bridge.call(...)` calls through application code.
- For a new Bridge operation, update all four contract points in one change: the MCU implementation, `Bridge.provide_safe(...)` registration, the typed `MiniAutoRobot` wrapper, and its call site or documented payload.
- Make routines interruptible through the existing program-enabled flow. Use `run_program(...)`, guarded robot operations, and `finally: robot.stop()` for application entry points that can move hardware.
- Keep the current health and sensor fields backward compatible. Add fields instead of silently renaming or changing their units.
- Do not embed model binaries or generated inference artifacts in this driver repository; `.gitignore` intentionally excludes common model formats.
- Never run commands that move motors, actuate the servo, flash boards, or alter a live robot unless the user has explicitly requested hardware execution and confirmed a safe physical setup.

## Validate in stages

Run checks that do not require hardware first:

```bash
python3 -m py_compile python/*.py
python3 .agents/skills/uno-q-miniauto/scripts/check_robot_contract.py --strict
```

Then:

1. Compile the UNO Q sketch with the repository's `arduino:zephyr` profile when that toolchain is available.
2. Compile the ESP32-S3 camera sketch only when camera code changed and its board libraries are available.
3. If hardware testing is authorized, start with wheels raised, low speed, short timed pulses, and an immediately reachable stop control.
4. Test one motor or actuator at a time before combined motion.
5. Verify `health()`, `read_sensors()`, program enable/disable, interruption, and final stop behavior before testing autonomous logic.

Do not claim a hardware result from static checks or compilation. Report which layers changed, which checks passed, and which physical behaviors remain unverified.

## Keep the handoff usable

Update `README.md` when public APIs, setup, environment variables, commands, payloads, or camera endpoints change. Keep `docs/MANUAL.md` focused on physical assembly and battery safety. Finish with a small usage example for the new variant and explicit first-run safety instructions.
