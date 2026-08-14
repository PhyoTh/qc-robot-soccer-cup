"""
play_match.py  -  match-day App Lab entry point (AI-driven autonomous play).

This is a SEPARATE, OPTIONAL entry point - it does not touch python/main.py
or app.yaml. The team deliberately keeps app.yaml pointed at python/main.py
(the hardware bring-up demo) until this file has been validated against the
real robot, camera stream, and an exported model tomorrow at the venue. Once
that validation passes, switch app.yaml's entry point to this file.

Wires together the pieces built tonight:
  - robot_client.MiniAutoRobot   - Bridge RPCs to the UNO Q firmware
  - ei_runner.ObjectDetector     - FOMO model (goal / robot / soccer_ball)
  - wall_detector.WallSideDetector - HSV wall-tape colour classification
  - soccer_policy.SoccerPolicy   - the Decide -> Act tick, including the
                                    own-goal-avoidance strategy

Follows python/main.py's exact App.run(user_loop=lambda: robot.run_program(...))
integration pattern, with robot.stop() in a finally block.
"""
import os
import sys
import time

from soccer_policy import SoccerPolicy
from wall_detector import WallSideDetector

# arduino.app_utils and robot_client are only importable on-device (UNO Q
# Linux side, App Lab runtime) - robot_client.py itself does an unconditional
# `from arduino.app_utils import Bridge` at module load, so a plain laptop
# import of either raises ImportError. Soft-import both here so this file
# still imports and py_compiles cleanly on a laptop tonight; main() checks
# for None and prints a clear actionable message instead of a raw traceback
# if it's ever actually run somewhere these aren't installed.
try:
    from arduino.app_utils import App
except ImportError as exc:
    App = None
    _APP_UTILS_IMPORT_ERROR = exc
else:
    _APP_UTILS_IMPORT_ERROR = None

try:
    from robot_client import MiniAutoRobot
except ImportError as exc:
    MiniAutoRobot = None
    _ROBOT_CLIENT_IMPORT_ERROR = exc
else:
    _ROBOT_CLIENT_IMPORT_ERROR = None

try:
    import cv2
except ImportError as exc:  # cv2 only guaranteed on-device, not on a plain laptop
    cv2 = None
    _CV_IMPORT_ERROR = exc
else:
    _CV_IMPORT_ERROR = None

import ei_runner  # already laptop-safe: soft-imports edge_impulse_linux/cv2/numpy itself

# --- Match-day config -------------------------------------------------
CAMERA_URL = "http://192.168.5.1:81/stream"  # ESP32-S3 MJPEG stream, per README's Camera stream section
CAMERA_WARMUP_MAX_FRAMES = 50   # discard empty frames on startup, per README's warmup note
CAMERA_WARMUP_RETRY_DELAY = 0.05

# Overridable at match time without editing code, e.g. once the model is
# exported tonight/tomorrow to a different filename:
#   EI_MODEL_PATH=models/soccer-linux-aarch64.eim python3 python/play_match.py
DEFAULT_MODEL_PATH = "models/soccer-linux-aarch64.eim"

MAX_MATCH_SECONDS = 6 * 60  # bounded safety ceiling; a whistle/BOOT stop should end play sooner


def _open_camera():
    """Open the MJPEG stream and warm it up, discarding empty frames until a
    real one arrives - exactly as README.md's Wall / Field-Side Detection
    section documents (short warmup loop before running detection)."""
    if cv2 is None:
        raise RuntimeError(
            "opencv-python is not installed - run: pip install opencv-python"
        ) from _CV_IMPORT_ERROR

    print(f"[INFO] connecting to camera: {CAMERA_URL}")
    cap = cv2.VideoCapture(CAMERA_URL)

    for attempt in range(CAMERA_WARMUP_MAX_FRAMES):
        ok, frame = cap.read()
        if ok and frame is not None and frame.size > 0:
            print(f"[INFO] camera warmed up after {attempt + 1} frame(s)")
            return cap
        time.sleep(CAMERA_WARMUP_RETRY_DELAY)

    cap.release()
    raise RuntimeError(
        f"camera did not produce a usable frame after {CAMERA_WARMUP_MAX_FRAMES} attempts "
        f"- check the camera is powered and connected to {CAMERA_URL}"
    )


def _build_tick(robot: MiniAutoRobot, cap, detector, wall: WallSideDetector):
    policy = SoccerPolicy(robot)

    def tick() -> None:
        """One Acquire -> Preprocess -> ... -> Reobserve iteration. Bounded
        loop matching precision_course.py's structure: read fresh state,
        act at most once, loop back and reobserve - never a long blocking
        sequence, so a BOOT-button stop can interrupt between ticks."""
        from robot_client import ProgramStopped  # local import: keeps module import laptop-safe

        # Every fresh session (each BOOT-button re-enable) may follow a
        # Yellow Card reset, a ref-initiated pause, or just a practice
        # restart - the robot and ball have very likely been physically
        # repositioned, so clear any remembered ball tracking/search state
        # from before rather than act on a world that no longer exists.
        policy.reset()

        start = time.monotonic()
        try:
            while (time.monotonic() - start) < MAX_MATCH_SECONDS and robot.is_running():
                sensors = robot.read_sensors()

                ok, frame = cap.read()
                frame_ts = time.monotonic()
                if not ok or frame is None or frame.size == 0:
                    frame = None

                policy.decide_and_act(frame, sensors, detector, wall, robot.hold_toggle(), frame_ts=frame_ts)
        except ProgramStopped:
            print("[INFO] match stopped - BOOT button disabled the program")

    return tick


def main() -> None:
    if MiniAutoRobot is None or App is None:
        # Expected tonight on a plain laptop - these only exist inside the
        # UNO Q App Lab runtime. Print one clear line instead of a raw
        # traceback and exit cleanly so this still serves as a self-test.
        missing = _ROBOT_CLIENT_IMPORT_ERROR or _APP_UTILS_IMPORT_ERROR
        print(f"[WARN] arduino.app_utils / robot_client not available ({missing})")
        print("[INFO] play_match.py must run on-device (UNO Q App Lab runtime) - skipping")
        return

    robot = MiniAutoRobot()

    print(f"health   : {robot.health()}")
    print(f"sensors  : {robot.read_sensors()}")
    team = "BLUE" if robot.hold_toggle() else "RED"
    print(f"[TEAM] active team: {team}  (hold CAM button 5 s to switch)")

    model_path = os.environ.get("EI_MODEL_PATH", DEFAULT_MODEL_PATH)

    try:
        cap = _open_camera()
    except RuntimeError as exc:
        print(f"[WARN] camera unavailable - {exc}")
        print("[INFO] play_match.py requires the camera; exiting instead of running blind")
        return

    wall = WallSideDetector()

    try:
        # See main.py: must stay at or below soccer_policy's lowest per-class
        # floor, since this filter runs before the policy's own thresholds.
        with ei_runner.ObjectDetector(model_path, min_confidence=0.35) as detector:
            print("[INFO] model loaded - starting match loop")
            print("[INFO] waiting for BOOT button to start...")
            tick = _build_tick(robot, cap, detector, wall)
            try:
                App.run(user_loop=lambda: robot.run_program(tick))
            finally:
                robot.stop()
    except ei_runner.ModelUnavailable as exc:
        print(f"[WARN] model unavailable - {exc}")
        print("[INFO] export the model (see README.md's 'Model Import' section) and retry")
        sys.exit(1)  # finally below still runs and releases the camera before exiting
    finally:
        cap.release()


if __name__ == "__main__":
    main()
