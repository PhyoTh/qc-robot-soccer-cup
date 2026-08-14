"""
test_vision.py  -  live detection viewer. NO motor movement, safe to run anywhere.

Written at the venue to answer one question fast: "what does the model
actually see right now?" diagnostics.py only runs a single frame; this
loops continuously so you can carry the robot around, point it at the
ball / goal / another robot, and watch detections stream by in real time.

Also prints the wall-tape HSV coverage every frame, so it doubles as the
calibration tool - point the camera at the red or blue tape and watch
whether the percentages actually move.

Run (inside the container):
    python3 python/test_vision.py --model models/pico-160.eim
    python3 python/test_vision.py --model models/nano-192.eim --min-confidence 0.3

Ctrl+C to stop. Never drives the motors, so wheels-down is fine.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

CAMERA_URL = "http://192.168.5.1:81/stream"
DEFAULT_MODEL_PATH = "models/pico-160.eim"
CAMERA_WARMUP_MAX_FRAMES = 50
CAMERA_WARMUP_RETRY_DELAY = 0.05


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=os.environ.get("EI_MODEL_PATH", DEFAULT_MODEL_PATH))
    parser.add_argument(
        "--min-confidence",
        type=float,
        default=0.3,
        help="Deliberately lower than soccer_policy's 0.6 so you can SEE weak detections "
        "that the policy would currently reject - useful for deciding whether the real "
        "problem is the model or just the threshold.",
    )
    parser.add_argument("--interval", type=float, default=0.3, help="seconds between frames")
    args = parser.parse_args()

    try:
        import cv2
    except ImportError:
        print("[FAIL] opencv-python not installed")
        sys.exit(1)

    try:
        import ei_runner
        from wall_detector import WallSideDetector
    except ImportError as exc:
        print(f"[FAIL] could not import project modules ({exc}) - run this from the repo root")
        sys.exit(1)

    print(f"[INFO] connecting to camera: {CAMERA_URL}")
    cap = cv2.VideoCapture(CAMERA_URL)
    for attempt in range(CAMERA_WARMUP_MAX_FRAMES):
        ok, frame = cap.read()
        if ok and frame is not None and frame.size > 0:
            print(f"[INFO] camera ready after {attempt + 1} frame(s), shape={frame.shape}")
            break
        time.sleep(CAMERA_WARMUP_RETRY_DELAY)
    else:
        print("[FAIL] camera never produced a usable frame")
        cap.release()
        sys.exit(1)

    wall = WallSideDetector()

    try:
        with ei_runner.ObjectDetector(args.model, min_confidence=args.min_confidence) as detector:
            print(f"[INFO] model loaded: {args.model}")
            print(f"[INFO] showing detections above confidence {args.min_confidence}")
            print("[INFO] Ctrl+C to stop\n")

            frame_count = 0
            while True:
                ok, frame = cap.read()
                if not ok or frame is None or frame.size == 0:
                    print("[WARN] dropped frame")
                    time.sleep(args.interval)
                    continue

                frame_count += 1
                frame_w = frame.shape[1]

                try:
                    detections = detector.infer(frame)
                except Exception as exc:  # noqa: BLE001 - keep the viewer alive through one bad frame
                    print(f"[WARN] inference failed: {exc}")
                    time.sleep(args.interval)
                    continue

                analysis = wall.analyze(frame)
                print(f"--- frame {frame_count} | {wall.diagnostic_line(analysis)}")

                if not detections:
                    print("    (nothing detected)")
                else:
                    for d in detections:
                        # Where it is horizontally, in the same terms soccer_policy uses
                        # to decide turn-vs-approach, so what you see here maps directly
                        # onto what the policy would do.
                        offset = d.center_x - frame_w / 2.0
                        side = "LEFT " if offset < 0 else "RIGHT"
                        size_frac = d.width / frame_w
                        print(
                            f"    {d.label:12s} conf={d.confidence:.2f}  "
                            f"{side} off={abs(offset):5.0f}px  size={size_frac:.0%} of frame"
                        )

                time.sleep(args.interval)
    except ei_runner.ModelUnavailable as exc:
        print(f"[FAIL] {exc}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n[INFO] stopped")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
