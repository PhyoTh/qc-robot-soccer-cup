"""
diagnostics.py - one-command morning bring-up check.

Tomorrow's dev window is 5 hours. README.md's bring-up stages (health, Bridge
status, sensor validity, camera stream, wall-tape calibration, model loading)
are each individually documented, but checking them one at a time costs real
minutes the team doesn't have to spare. This script runs all of them in one
pass and prints a clear PASS/FAIL/SKIP per check, so a single "run this
first" command replaces five separate manual steps.

Every check is independent and wrapped so one failure doesn't block the
rest - if the camera isn't plugged in yet, you still get the robot's health
and sensor status; if no model has been exported yet, you still get
everything else. Run it again after fixing whatever failed.

Usage (on the UNO Q's Linux side, after building/running the App Lab app):
    python3 python/diagnostics.py
    python3 python/diagnostics.py --model models/soccer-pico-160.eim
    python3 python/diagnostics.py --wall-seconds 10   # longer HSV calibration read
"""
from __future__ import annotations

import argparse
import os
import time

CAMERA_URL = "http://192.168.5.1:81/stream"
DEFAULT_MODEL_PATH = "models/soccer-linux-aarch64.eim"
CAMERA_WARMUP_MAX_FRAMES = 50
CAMERA_WARMUP_RETRY_DELAY = 0.05


def _header(title: str) -> None:
    print(f"\n=== {title} ===")


def _pass(msg: str) -> None:
    print(f"  [PASS] {msg}")


def _fail(msg: str) -> None:
    print(f"  [FAIL] {msg}")


def _skip(msg: str) -> None:
    print(f"  [SKIP] {msg}")


def check_robot():
    """Bridge/health/sensor check. Returns (robot, sensors) or (None, None)."""
    _header("Robot (health + sensors)")
    try:
        from robot_client import MiniAutoRobot
    except ImportError as exc:
        _skip(f"robot_client not importable ({exc}) - not running on the UNO Q Linux side?")
        return None, None

    try:
        robot = MiniAutoRobot()
        health = robot.health()
        if health.get("bridge") and health.get("serial"):
            _pass(f"health: {health}")
        else:
            _fail(f"health reported bridge/serial not both true: {health}")

        sensors = robot.read_sensors()
        if not sensors:
            _fail("read_sensors() returned empty - Bridge not responding")
            return robot, {}

        if sensors.get("line_ok"):
            _pass(f"line sensor OK: line_digital={sensors.get('line_digital')}")
        else:
            _fail("line sensor read invalid (line_ok=False) - check I2C wiring at 0x78")

        ultrasonic_mm = sensors.get("ultrasonic_mm", -1)
        if ultrasonic_mm > 0:
            _pass(f"ultrasonic OK: {ultrasonic_mm}mm")
        else:
            _fail("ultrasonic read invalid (-1) - check I2C wiring at 0x77")

        battery_mv = sensors.get("battery_mv", 0)
        if battery_mv > 6000:  # rough sanity floor, not a precision threshold
            _pass(f"battery: {battery_mv}mV")
        else:
            _fail(f"battery reads low or missing: {battery_mv}mV - charge before match play")

        team = "BLUE" if sensors.get("hold_toggle") else "RED"
        print(f"  [INFO] active team: {team} (hold CAM button 5s to switch)")
        print(f"  [INFO] program_enabled: {sensors.get('program_enabled')}")

        return robot, sensors
    except Exception as exc:  # noqa: BLE001 - a diagnostic must never itself crash the check run
        _fail(f"unexpected error talking to the robot: {exc}")
        return None, None


def check_camera():
    """Camera stream check. Returns an open cv2.VideoCapture, or None."""
    _header("Camera stream")
    try:
        import cv2
    except ImportError:
        _skip("opencv-python not installed - run: pip install opencv-python")
        return None

    print(f"  [INFO] connecting to {CAMERA_URL} ...")
    cap = cv2.VideoCapture(CAMERA_URL)
    for attempt in range(CAMERA_WARMUP_MAX_FRAMES):
        ok, frame = cap.read()
        if ok and frame is not None and frame.size > 0:
            _pass(f"frame received after {attempt + 1} attempt(s), shape={frame.shape}")
            return cap
        time.sleep(CAMERA_WARMUP_RETRY_DELAY)

    _fail(f"no usable frame after {CAMERA_WARMUP_MAX_FRAMES} attempts - is the camera powered and joined?")
    cap.release()
    return None


def check_wall(cap, seconds: float) -> None:
    """Live HSV calibration read - prints diagnostic_line() for `seconds`,
    exactly the "print coverage percentages at startup" tip from README.md."""
    _header(f"Wall/field-tape calibration ({seconds:.0f}s live read)")
    if cap is None:
        _skip("no camera available")
        return
    try:
        from wall_detector import WallSideDetector
    except ImportError as exc:
        _skip(f"wall_detector not importable ({exc})")
        return

    detector = WallSideDetector()
    start = time.monotonic()
    reads = 0
    try:
        while (time.monotonic() - start) < seconds:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            analysis = detector.analyze(frame)
            print(f"  {detector.diagnostic_line(analysis)}")
            reads += 1
            time.sleep(0.2)
    except Exception as exc:  # noqa: BLE001
        _fail(f"error running wall_detector against live frames: {exc}")
        return

    if reads > 0:
        _pass(
            f"{reads} live read(s) completed - point the camera at a KNOWN end and check the "
            f"'facing' verdict, not the percentages. The verdict comes from which colour's "
            f"off= (centroid distance from frame centre) is SMALLER; percentages only show "
            f"whether tape was seen at all. If tape is visible but both px counts are ~0, "
            f"loosen sat_min/val_min; if the verdict is wrong while both colours are seen, "
            f"check band_top_frac/band_bottom_frac in wall_detector.py."
        )
    else:
        _fail("no frames read during the calibration window")


def check_model(model_path: str, cap) -> None:
    """Model load + one live inference if a camera frame is available."""
    _header(f"Model ({model_path})")
    if not os.path.isfile(model_path):
        _skip(f"file not found - export from Edge Impulse Studio and place it here, or pass --model")
        return

    try:
        import ei_runner
    except ImportError as exc:
        _skip(f"ei_runner not importable ({exc})")
        return

    try:
        with ei_runner.ObjectDetector(model_path) as detector:
            _pass(f"model loaded - labels={detector.labels}")

            if cap is None:
                _skip("no camera available - skipping live inference smoke test")
                return
            ok, frame = cap.read()
            if not ok or frame is None:
                _fail("could not read a frame for the inference smoke test")
                return
            detections = detector.infer(frame)
            if detections:
                for d in detections:
                    print(f"  [INFO] {d.label} conf={d.confidence:.2f} xy=({d.x},{d.y}) wh=({d.width},{d.height})")
                _pass(f"{len(detections)} detection(s) on a live frame")
            else:
                _pass("model ran with no detections on this frame (point it at the ball/goal/an opponent and rerun)")
    except ei_runner.ModelUnavailable as exc:
        _fail(str(exc))
    except Exception as exc:  # noqa: BLE001
        _fail(f"unexpected error running the model: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=os.environ.get("EI_MODEL_PATH", DEFAULT_MODEL_PATH))
    parser.add_argument("--wall-seconds", type=float, default=5.0)
    args = parser.parse_args()

    print("Robot Soccer Cup - morning bring-up diagnostics")

    _, _sensors = check_robot()
    cap = check_camera()
    check_wall(cap, args.wall_seconds)
    check_model(args.model, cap)

    if cap is not None:
        cap.release()

    print("\nDone. Re-run after fixing anything marked [FAIL] - each check is independent.")


if __name__ == "__main__":
    main()
