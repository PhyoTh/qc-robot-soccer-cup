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
# Consecutive dead frames before the match loop tears the stream down and
# reopens it. The ESP32-S3 MJPEG stream drops under load - observed live as
# "Stream ends prematurely" - and cv2.VideoCapture never recovers on its own.
# Low enough to lose well under a second of play, high enough that a couple
# of dropped frames don't cause a needless reconnect (which costs ~1 tick).
CAMERA_RECONNECT_AFTER_BAD_FRAMES = 5
# Warmup budget for a MID-MATCH reconnect. Short on purpose: standing still
# for the full 2.5s warmup mid-game is worse than failing fast and retrying
# on the next tick.
CAMERA_RECONNECT_WARMUP_FRAMES = 10
MAX_MATCH_SECONDS = 6 * 60


def _setting(name: str, default: str = "") -> str:
    """Read a setting from the environment, falling back to a KEY=VALUE line
    in <app root>/match_config.txt.

    The file exists because App Lab's Run button may not offer a way to set
    environment variables, and stopping the app to run things by hand is not
    an option either - stopping it destroys the container that holds the
    arduino runtime. So the file is the reliable channel: edit it from the
    host, hit Run, done.
    """
    from_env = os.environ.get(name)
    if from_env is not None and from_env.strip():
        return from_env.strip()

    config = APP_ROOT / "match_config.txt"
    if config.is_file():
        try:
            for line in config.read_text().splitlines():
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, _, value = line.partition("=")
                if key.strip().upper() == name.upper():
                    return value.strip()
        except OSError as exc:
            print(f"[WARN] could not read {config}: {exc}")
    return default


def _team_is_blue(robot):
    """Resolve which team we're on, as a callable read fresh each tick.

    TEAM=auto (default) uses the CAM button's hold_toggle, which is the
    mechanism the organizers specified. TEAM=red / TEAM=blue overrides it
    outright - useful because the button is easy to get wrong under pressure
    (it needs a 5-second hold, and the confirming LED is one of two lights on
    the robot), whereas a config line is unambiguous and survives reboots.

    Either way the robot ends up knowing its colour, which is what the rule
    is actually about. Getting this wrong makes the robot attack its own net,
    so whichever source is in use gets printed in the startup banner.
    """
    setting = _setting("TEAM", "auto").lower()
    if setting in ("red", "r", "false", "0"):
        return (lambda: False), "RED (forced by match_config)"
    if setting in ("blue", "b", "true", "1"):
        return (lambda: True), "BLUE (forced by match_config)"
    if setting not in ("auto", ""):
        print(f"[WARN] TEAM={setting!r} not understood - falling back to the CAM button")
    current = "BLUE" if robot.hold_toggle() else "RED"
    return robot.hold_toggle, f"{current} (from CAM button - hold 5s to change)"


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


def _open_camera(max_frames: int = CAMERA_WARMUP_MAX_FRAMES):
    """Open the MJPEG stream, discarding empty frames until a real one lands.

    max_frames is lowered for mid-match reconnects: the full warmup blocks for
    ~2.5s, which is a long time to stand still during a 5-minute game. A
    reconnect that doesn't take immediately is better retried next tick than
    waited on.
    """
    import cv2

    print(f"[INFO] connecting to camera: {CAMERA_URL}")
    cap = cv2.VideoCapture(CAMERA_URL)
    for attempt in range(max_frames):
        ok, frame = cap.read()
        if ok and frame is not None and frame.size > 0:
            print(f"[INFO] camera warmed up after {attempt + 1} frame(s)")
            return cap
        time.sleep(CAMERA_WARMUP_RETRY_DELAY)
    cap.release()
    raise RuntimeError(f"camera produced no usable frame after {max_frames} attempts")


def run_match(robot, App, policy_class=None, team_fn=None) -> None:
    """Autonomous match play. policy_class lets trickshot reuse this loop.
    team_fn returns True for BLUE - see _team_is_blue."""
    import ei_runner
    from robot_client import ProgramStopped
    from soccer_policy import SoccerPolicy
    from wall_detector import WallSideDetector

    model_path = _setting("EI_MODEL_PATH", DEFAULT_MODEL_PATH)
    policy_class = policy_class or SoccerPolicy
    team_fn = team_fn or robot.hold_toggle

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

            # CELEBRATE=1 fires the Redemption Cup celebration whenever the
            # policy believes it scored. OFF by default for bracket play:
            # celebrating burns match clock for no points, and a false
            # positive would do it while the ball is still live.
            on_goal = None
            if _setting("CELEBRATE").lower() in ("1", "true", "yes", "on"):
                from celebration import celebrate
                on_goal = lambda: celebrate(robot, cycles=1)  # noqa: E731
                print("[INFO] celebration ENABLED - will fire on a detected goal")

            policy = policy_class(robot, on_goal_scored=on_goal)
            print("[INFO] waiting for BOOT button to start...")

            def tick() -> None:
                # Fresh session: a Yellow Card reset or a ref pause very
                # likely repositioned the robot and ball, so drop any
                # remembered tracking state rather than acting on a world
                # that no longer exists.
                nonlocal cap
                policy.reset()
                start = time.monotonic()
                bad_frames = 0
                try:
                    while (time.monotonic() - start) < MAX_MATCH_SECONDS and robot.is_running():
                        sensors = robot.read_sensors()
                        ok, frame = cap.read()
                        frame_ts = time.monotonic()
                        if not ok or frame is None or frame.size == 0:
                            frame = None
                            bad_frames += 1
                            # The ESP32-S3 MJPEG stream DOES drop mid-match -
                            # seen live as "Stream ends prematurely". Once
                            # that happens cap.read() fails forever, so
                            # without reopening the socket the robot goes
                            # permanently blind and just skips ticks for the
                            # rest of the game. capture.py has always
                            # reconnected for this reason; the match loop
                            # must too.
                            if bad_frames >= CAMERA_RECONNECT_AFTER_BAD_FRAMES:
                                print(f"[WARN] {bad_frames} dead frames - reopening the camera stream")
                                try:
                                    cap.release()
                                except Exception:  # noqa: BLE001
                                    pass
                                try:
                                    cap = _open_camera(max_frames=CAMERA_RECONNECT_WARMUP_FRAMES)
                                    print("[INFO] camera stream recovered")
                                except Exception as exc:  # noqa: BLE001 - keep playing blind-but-alive
                                    print(f"[WARN] camera reopen failed ({exc}) - will retry")
                                bad_frames = 0
                        else:
                            bad_frames = 0
                        policy.decide_and_act(
                            frame, sensors, detector, wall, team_fn(), frame_ts=frame_ts
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

    model_path = _setting("EI_MODEL_PATH", DEFAULT_MODEL_PATH)
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
    mode = _setting("MODE", "match").lower()
    print(f"[INFO] MODE={mode}")

    if mode in ("match", "vision", "trickshot", "fastball"):
        _ensure_edge_impulse()

    from arduino.app_utils import App
    from robot_client import MiniAutoRobot

    robot = MiniAutoRobot()
    print(f"health   : {robot.health()}")
    sensors = robot.read_sensors()
    print(f"sensors  : {sensors}")
    # The referee assigns red or blue PER ROUND and it can change between
    # rounds, so this must be verified before every single game. It is what
    # own-goal avoidance compares the field tape against - if it is wrong the
    # robot will confidently attack its own net. Printed as a banner because
    # it is the one setting that fails silently.
    team_fn, team_desc = _team_is_blue(robot)
    print("=" * 58)
    print(f"   TEAM = {team_desc}")
    print("   Wrong? set TEAM=red or TEAM=blue in match_config.txt")
    print("          (or TEAM=auto to use the CAM button's 5s hold)")
    print("   Check this before EVERY round - the ref can reassign it.")
    print("=" * 58)

    # Which corner the referee placed us in decides which way the kickoff
    # burst strafes. Backwards means driving into the corner instead of onto
    # the goal line, so it is banner-printed alongside the team colour.
    import soccer_policy as _sp
    corner = _setting("START_CORNER", _sp.OPENING_START_CORNER).lower()
    try:
        _sp.set_opening_corner(corner)
    except ValueError as exc:
        print(f"[WARN] {exc} - keeping {_sp.OPENING_START_CORNER}")
    if _sp.OPENING_SEQUENCE:
        _side, _spd, _ms = _sp.OPENING_SEQUENCE[0]
        print(f"   START CORNER = {_sp.OPENING_START_CORNER.upper()}  -> opening strafes {_side.upper()} ({_ms}ms)")
        print("   Wrong? re-run with START_CORNER=left  (or =right)")
        print("=" * 58)

    if not sensors.get("line_ok"):
        print("[WARN] line sensor unavailable - precision course will not work, match play is unaffected")
    if sensors.get("ultrasonic_mm", -1) <= 0:
        print("[WARN] ultrasonic unavailable - opponent foul-avoidance disabled, match play continues")

    try:
        if mode == "match":
            run_match(robot, App, team_fn=team_fn)
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
            run_match(robot, App, policy_class=TrickShotPolicy, team_fn=team_fn)
        elif mode == "fastball":
            run_fastball(robot, App)
        elif mode == "idle":
            # Keeps the container alive while claiming NOTHING - no BOOT
            # handler, no motors. Needed because stopping the App Lab app
            # destroys the container, and the container is the only place
            # arduino.app_utils exists. So to run something by hand (e.g.
            # calibrate_motion.py) you leave the app running in idle and
            # exec into it, instead of stopping it and losing the runtime.
            print("[INFO] IDLE - container alive, BOOT button free for manual scripts.")
            print("[INFO] docker exec -it miniautodriver-main-1 bash   then run what you want.")
            while True:
                time.sleep(3600)
        else:
            print(f"[FAIL] unknown MODE={mode!r} - see this file's docstring for valid values")
    finally:
        robot.stop()


def run_fastball(robot, App) -> None:
    """Redemption Cup fastest-ball-detection challenge."""
    import ei_runner
    from fastest_ball_detection import run_fastest_ball_detection

    model_path = _setting("EI_MODEL_PATH", DEFAULT_MODEL_PATH)
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
