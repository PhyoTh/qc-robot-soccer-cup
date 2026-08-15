"""
test_rotate.py  -  does the robot actually ROTATE and find the ball?

Isolates the search behaviour from everything else. No opening burst, no
pushing, no goal logic - it only sweeps and reports what it sees, so if the
robot fails to find a ball here the problem is rotation or detection, not
policy. Uses the same drive() calls, speed and durations as the real search
in soccer_policy, so what you watch here is what happens in a match.

Run (inside the app container, after MODE=idle + Run so the BOOT button is
free):
    cd /app
    python3 python/test_rotate.py                  # rotate + look for the ball
    python3 python/test_rotate.py --no-model       # rotate ONLY, no camera
    python3 python/test_rotate.py --sweeps 4       # more sweeps before giving up
    python3 python/test_rotate.py --speed 160      # try a faster rotation

--no-model is the first thing to try if you are unsure the robot turns at
all: it removes the camera and model entirely, so anything that goes wrong
is mechanical.

WHEELS DOWN, clear space. Ctrl+C stops. Every pulse is bounded and the
robot is stopped on the way out.
"""
from __future__ import annotations

import argparse
import sys
import time

DEFAULT_MODEL_PATH = "models/pico-160.eim"
CAMERA_URL = "http://192.168.5.1:81/stream"


def _wait_for_enable(robot, timeout_s: float = 120.0) -> bool:
    if robot.is_running():
        return True
    print()
    print("  >>> PRESS THE BOOT BUTTON (short press, on the camera module).")
    print("      Wait for red -> yellow -> solid green.")
    print("      If App Lab is running MODE=match it will steal the press -")
    print("      set MODE=idle in match_config.txt and Run again first.")
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        if robot.is_running():
            print("  program enabled.\n")
            return True
        time.sleep(0.2)
    print(f"  [FAIL] no BOOT press within {timeout_s:.0f}s")
    return False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=DEFAULT_MODEL_PATH)
    parser.add_argument("--no-model", action="store_true", help="rotate only, no camera/model")
    parser.add_argument("--sweeps", type=int, default=2, help="how many sweeps before giving up")
    parser.add_argument("--speed", type=int, default=None, help="override rotation speed")
    parser.add_argument("--ms", type=int, default=None, help="override pulse duration")
    args = parser.parse_args()

    try:
        from robot_client import MiniAutoRobot, ProgramStopped
        import soccer_policy as sp
    except ImportError as exc:
        print(f"[FAIL] run this from /app inside the container: {exc}")
        sys.exit(1)

    speed = args.speed if args.speed is not None else sp.SEARCH_SPEED
    pulse_ms = args.ms if args.ms is not None else sp.TURN_MS
    sweep_ticks = sp.SEARCH_SWEEP_TICKS

    print("=" * 62)
    print("  ROTATION TEST - wheels down, clear space")
    print(f"  speed={speed}  pulse={pulse_ms}ms  sweep={sweep_ticks} ticks/direction")
    print(f"  = {sweep_ticks * pulse_ms}ms of continuous rotation per sweep")
    print("=" * 62)

    detector = None
    cap = None
    if not args.no_model:
        try:
            import cv2
            import ei_runner

            print(f"[INFO] opening camera {CAMERA_URL}")
            cap = cv2.VideoCapture(CAMERA_URL)
            for _ in range(50):
                ok, frame = cap.read()
                if ok and frame is not None and frame.size > 0:
                    break
                time.sleep(0.05)
            else:
                print("[WARN] camera gave no frames - continuing WITHOUT detection")
                cap.release()
                cap = None
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] camera setup failed ({exc}) - continuing WITHOUT detection")
            cap = None

    robot = MiniAutoRobot()
    if not _wait_for_enable(robot):
        sys.exit(1)

    ctx = None
    try:
        if cap is not None:
            import ei_runner

            ctx = ei_runner.ObjectDetector(args.model, min_confidence=0.35)
            detector = ctx.__enter__()
            print(f"[INFO] model loaded: {args.model}\n")
    except Exception as exc:  # noqa: BLE001
        print(f"[WARN] model failed to load ({exc}) - rotating without detection\n")
        detector = None

    turn_left = True
    tick = 0
    found = False
    try:
        for sweep in range(args.sweeps * 2):  # each "sweep" is one direction
            direction = "rotate_left" if turn_left else "rotate_right"
            print(f"--- sweep {sweep + 1}: {direction} x{sweep_ticks} ---")
            for i in range(sweep_ticks):
                if not robot.is_running():
                    print("[INFO] program disabled - stopping")
                    return
                tick += 1

                seen = ""
                if detector is not None and cap is not None:
                    ok, frame = cap.read()
                    if ok and frame is not None and frame.size > 0:
                        try:
                            dets = detector.infer(frame)
                            ball = detector.best(dets, "soccer_ball")
                            if ball is not None:
                                offset = ball.center_x - frame.shape[1] / 2.0
                                seen = f"  <<< BALL conf={ball.confidence:.2f} off={offset:+.0f}px"
                                found = True
                        except Exception as exc:  # noqa: BLE001
                            seen = f"  (inference error: {exc})"
                    else:
                        seen = "  (no frame)"

                print(f"  tick {tick:3d}  {direction}{seen}")
                robot.drive(direction, speed=speed, ms=pulse_ms)

                if found:
                    print("\n[RESULT] BALL FOUND - rotation + detection both working.")
                    return
            turn_left = not turn_left

        print(f"\n[RESULT] completed {args.sweeps} full sweeps without seeing the ball.")
        if detector is None:
            print("         (detection was OFF - this only tested that it turns)")
        else:
            print("         Rotation ran; either no ball was in view or detection is failing.")
            print("         Try: python3 python/test_vision.py --model " + args.model)
    except ProgramStopped:
        print("\n[INFO] BOOT button stopped the program")
    except KeyboardInterrupt:
        print("\n[INFO] interrupted")
    finally:
        robot.stop()
        if ctx is not None:
            try:
                ctx.__exit__(None, None, None)
            except Exception:  # noqa: BLE001
                pass
        if cap is not None:
            cap.release()
        print("[INFO] stopped.")


if __name__ == "__main__":
    main()
