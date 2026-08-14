"""
main.py  -  App Lab entry point. Hit RUN in App Lab and this is what runs.

Rewritten at the venue so the match code launches from App Lab's Run button
instead of needing an SSH session every time. The original hardware
bring-up demo is preserved as main_demo_backup.py - see MODE below to get
it back.

Two things this handles that a plain script wouldn't:

1. DEPENDENCY BOOTSTRAP. App Lab recreates the app container on Run, which
   wipes anything pip-installed into it. Rather than re-running pip by hand
   over SSH every single time, _ensure_edge_impulse() checks for the module
   and installs it from the .whl sitting in the app folder (which survives,
   because that folder is a host bind-mount). No network needed.

2. MODE SWITCH. Set the MODE env var to pick behaviour without editing
   code:
     MODE=match  (default) - autonomous soccer via soccer_policy
     MODE=vision           - print detections only, never moves the motors
     MODE=demo             - the original bring-up motion demo
     MODE=celebrate        - Redemption Cup celebration routine
     MODE=course           - Redemption Cup precision course
     MODE=trickshot        - Redemption Cup trick shot
     MODE=fastball         - Redemption Cup fastest ball detection

Safety is unchanged from play_match.py: nothing moves until the BOOT button
enables the program, a second press stops it, and robot.stop() runs in a
finally block on every path that can move hardware.
"""
import os
import subprocess
import sys
import time
from pathlib import Path

APP_ROOT = Path(__file__).resolve().parent.parent
CAMERA_URL = "http://192.168.5.1:81/stream"
DEFAULT_MODEL_PATH = "models/pico-160.eim"
CAMERA_WARMUP_MAX_FRAMES = 50
CAMERA_WARMUP_RETRY_DELAY = 0.05
MAX_MATCH_SECONDS = 6 * 60


def _ensure_edge_impulse() -> bool:
    """Install edge_impulse_linux from the bundled wheel if it's missing.

    App Lab recreates the container on every Run, wiping pip installs. The
    wheel lives in the app folder (a host bind-mount) so it always survives
    - installing from it takes a couple of seconds and needs no network,
    which matters because the robot sits on the camera's AP during matches
    and has no internet.
    """
    try:
        import edge_impulse_linux  # noqa: F401
        return True
    except ImportError:
        pass

    wheels = sorted(APP_ROOT.glob("edge_impulse_linux-*.whl"))
    if not wheels:
        print(f"[WARN] edge_impulse_linux missing and no wheel found in {APP_ROOT}")
        return False

    print(f"[INFO] bootstrapping edge_impulse_linux from {wheels[0].name}")
    result = subprocess.run(
        [sys.executable, "-m", "pip", "install", str(wheels[0])],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"[WARN] wheel install failed: {result.stderr.strip()[:300]}")
        return False

    # The freshly installed package landed in a user site-packages dir that
    # wasn't on sys.path when this process started - make it importable now
    # instead of forcing a restart.
    import site
    import importlib
    for path in site.getusersitepackages() if isinstance(site.getusersitepackages(), list) else [site.getusersitepackages()]:
        if path not in sys.path:
            sys.path.insert(0, path)
    importlib.invalidate_caches()

    try:
        import edge_impulse_linux  # noqa: F401
        print("[INFO] edge_impulse_linux ready")
        return True
    except ImportError as exc:
        print(f"[WARN] still cannot import edge_impulse_linux after install: {exc}")
        return False


def _open_camera():
    """Open the MJPEG stream, discarding empty frames until a real one lands."""
    import cv2

    print(f"[INFO] connecting to camera: {CAMERA_URL}")
    cap = cv2.VideoCapture(CAMERA_URL)
    for attempt in range(CAMERA_WARMUP_MAX_FRAMES):
        ok, frame = cap.read()
        if ok and frame is not None and frame.size > 0:
            print(f"[INFO] camera warmed up after {attempt + 1} frame(s)")
            return cap
        time.sleep(CAMERA_WARMUP_RETRY_DELAY)
    cap.release()
    raise RuntimeError(f"camera produced no usable frame after {CAMERA_WARMUP_MAX_FRAMES} attempts")


def run_match(robot, App, policy_class=None) -> None:
    """Autonomous match play. policy_class lets trickshot reuse this loop."""
    import ei_runner
    from robot_client import ProgramStopped
    from soccer_policy import SoccerPolicy
    from wall_detector import WallSideDetector

    model_path = os.environ.get("EI_MODEL_PATH", DEFAULT_MODEL_PATH)
    policy_class = policy_class or SoccerPolicy

    try:
        cap = _open_camera()
    except RuntimeError as exc:
        print(f"[FAIL] camera unavailable - {exc}")
        print("[INFO] check the robot is joined to the camera's Wi-Fi AP")
        return

    wall = WallSideDetector()

    try:
        # min_confidence here must stay at or below the LOWEST per-class floor
        # in soccer_policy.MIN_CONFIDENCE_BY_LABEL - this filter runs first, so
        # anything it drops can never reach the policy's own thresholds. The
        # policy does the real per-class filtering; this one just trims noise.
        with ei_runner.ObjectDetector(model_path, min_confidence=0.35) as detector:
            print(f"[INFO] model loaded: {model_path}")
            policy = policy_class(robot)
            print("[INFO] waiting for BOOT button to start...")

            def tick() -> None:
                # Fresh session: a Yellow Card reset or a ref pause very
                # likely repositioned the robot and ball, so drop any
                # remembered tracking state rather than acting on a world
                # that no longer exists.
                policy.reset()
                start = time.monotonic()
                try:
                    while (time.monotonic() - start) < MAX_MATCH_SECONDS and robot.is_running():
                        sensors = robot.read_sensors()
                        ok, frame = cap.read()
                        frame_ts = time.monotonic()
                        if not ok or frame is None or frame.size == 0:
                            frame = None
                        policy.decide_and_act(
                            frame, sensors, detector, wall, robot.hold_toggle(), frame_ts=frame_ts
                        )
                except ProgramStopped:
                    print("[INFO] stopped - BOOT button disabled the program")

            try:
                App.run(user_loop=lambda: robot.run_program(tick))
            finally:
                robot.stop()
    except ei_runner.ModelUnavailable as exc:
        print(f"[FAIL] model unavailable - {exc}")
    finally:
        cap.release()


def run_vision(robot) -> None:
    """Detection viewer - never touches the motors."""
    import cv2  # noqa: F401
    import ei_runner
    from wall_detector import WallSideDetector

    model_path = os.environ.get("EI_MODEL_PATH", DEFAULT_MODEL_PATH)
    try:
        cap = _open_camera()
    except RuntimeError as exc:
        print(f"[FAIL] camera unavailable - {exc}")
        return

    wall = WallSideDetector()
    try:
        with ei_runner.ObjectDetector(model_path, min_confidence=0.3) as detector:
            print(f"[INFO] model loaded: {model_path} (showing conf >= 0.3)")
            frame_count = 0
            while True:
                ok, frame = cap.read()
                if not ok or frame is None or frame.size == 0:
                    time.sleep(0.2)
                    continue
                frame_count += 1
                frame_w = frame.shape[1]
                detections = detector.infer(frame)
                print(f"--- frame {frame_count} | {wall.diagnostic_line(wall.analyze(frame))}")
                if not detections:
                    print("    (nothing detected)")
                for d in detections:
                    offset = d.center_x - frame_w / 2.0
                    side = "LEFT " if offset < 0 else "RIGHT"
                    print(
                        f"    {d.label:12s} conf={d.confidence:.2f}  {side} "
                        f"off={abs(offset):5.0f}px  size={d.width / frame_w:.0%}"
                    )
                time.sleep(0.3)
    except ei_runner.ModelUnavailable as exc:
        print(f"[FAIL] model unavailable - {exc}")
    except KeyboardInterrupt:
        print("[INFO] stopped")
    finally:
        cap.release()


def main() -> None:
    mode = os.environ.get("MODE", "match").strip().lower()
    print(f"[INFO] MODE={mode}")

    if mode in ("match", "vision", "trickshot", "fastball"):
        _ensure_edge_impulse()

    from arduino.app_utils import App
    from robot_client import MiniAutoRobot

    robot = MiniAutoRobot()
    print(f"health   : {robot.health()}")
    sensors = robot.read_sensors()
    print(f"sensors  : {sensors}")
    print(f"[TEAM] active team: {'BLUE' if robot.hold_toggle() else 'RED'}  (hold CAM button 5s to switch)")

    if not sensors.get("line_ok"):
        print("[WARN] line sensor unavailable - precision course will not work, match play is unaffected")
    if sensors.get("ultrasonic_mm", -1) <= 0:
        print("[WARN] ultrasonic unavailable - opponent foul-avoidance disabled, match play continues")

    try:
        if mode == "match":
            run_match(robot, App)
        elif mode == "vision":
            run_vision(robot)
        elif mode == "demo":
            import main_demo_backup  # noqa: F401  - running it is the point
        elif mode == "celebrate":
            from celebration import celebrate
            App.run(user_loop=lambda: robot.run_program(lambda: celebrate(robot)))
        elif mode == "course":
            from precision_course import run_precision_course
            App.run(user_loop=lambda: robot.run_program(lambda: run_precision_course(robot)))
        elif mode == "trickshot":
            from trick_shot import TrickShotPolicy
            run_match(robot, App, policy_class=TrickShotPolicy)
        elif mode == "fastball":
            run_fastball(robot, App)
        else:
            print(f"[FAIL] unknown MODE={mode!r} - see this file's docstring for valid values")
    finally:
        robot.stop()


def run_fastball(robot, App) -> None:
    """Redemption Cup fastest-ball-detection challenge."""
    import ei_runner
    from fastest_ball_detection import run_fastest_ball_detection

    model_path = os.environ.get("EI_MODEL_PATH", DEFAULT_MODEL_PATH)
    try:
        cap = _open_camera()
    except RuntimeError as exc:
        print(f"[FAIL] camera unavailable - {exc}")
        return

    try:
        with ei_runner.ObjectDetector(model_path) as detector:
            def frame_source():
                ok, frame = cap.read()
                return frame if ok and frame is not None and frame.size > 0 else None

            def routine() -> None:
                result = run_fastest_ball_detection(robot, detector, frame_source)
                print(f"[RESULT] {result}")

            print("[INFO] waiting for BOOT button to start the timed run...")
            App.run(user_loop=lambda: robot.run_program(routine))
    except ei_runner.ModelUnavailable as exc:
        print(f"[FAIL] model unavailable - {exc}")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
