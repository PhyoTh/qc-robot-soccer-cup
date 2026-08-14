"""
fastest_ball_detection.py - Redemption Cup -> "Fastest Ball Detection" challenge.

Judged purely by the elapsed time between challenge start and the moment the
robot initiates movement toward the detected ball. This is deliberately NOT
soccer_policy.py's decide_and_act() - there is no opponent, no goal, no
own-goal logic, and no wall_detector dependency to pay for in this event.
This module is a minimal, latency-focused "see ball, move toward ball, done"
loop: run_fastest_ball_detection() fires at most ONE robot.drive() call and
returns immediately.

Caveat worth remembering under time pressure: this module can only shrink the
polling loop *around* two costs it does not control - detector.infer()'s own
model inference time, and MiniAutoRobot.drive()'s internal ~100ms post-move
settle sleep (see robot_client.py's drive(): `time.sleep(ms / 1000.0 + 0.1)`
happens AFTER the Bridge.call that actually starts the motors, so it does not
delay "initiating movement" - but it's still latency baked into the on-device
stack that a judge's stopwatch may fold in). Nothing here can make either of
those faster; the only thing this module controls is how quickly it notices
a confident detection and calls drive() once that happens.

Only project-local, laptop-safe modules are referenced (ei_runner/robot_client
types are used for type hints only, under TYPE_CHECKING, exactly like
soccer_policy.py) so this file imports and self-tests with nothing but the
standard library installed.
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, Callable, Optional

if TYPE_CHECKING:
    from ei_runner import ObjectDetector
    from robot_client import MiniAutoRobot

# --- Tunables ---------------------------------------------------------------
# These two defaults are picked to MATCH soccer_policy.py's MIN_CONFIDENCE and
# CENTER_DEADZONE_FRAC for consistency across the repo's two decision loops -
# this module does not import those constants (no dependency wanted), so they
# are independently tunable here without touching match-play behaviour.
DEFAULT_MIN_CONFIDENCE = 0.6
DEFAULT_DEADZONE_FRAC = 0.12

APPROACH_SPEED = 150  # matches main.py / soccer_policy.py's default drive() speed
TURN_MS = 200         # bounded rotate pulse, matches soccer_policy.py's TURN_MS
DRIVE_MS = 250        # bounded forward pulse, matches soccer_policy.py's DRIVE_MS

# Sleep between empty polls (frame_source() returned None). Any sleep here
# adds directly to the judged latency, so keep it tiny - but a true 0-sleep
# busy loop would peg a CPU core and could starve the camera/Bridge threads
# that are trying to hand us the very frame we're waiting on. 10ms is a
# reasonable floor: negligible against a human-scale "fastest detection"
# result, but enough to yield the GIL/CPU between polls.
POLL_SLEEP_SEC = 0.01


def run_fastest_ball_detection(
    robot: "MiniAutoRobot",
    detector: "ObjectDetector",
    frame_source: Callable[[], Optional[Any]],
    min_confidence: float = DEFAULT_MIN_CONFIDENCE,
    max_seconds: float = 10.0,
    deadzone_frac: float = DEFAULT_DEADZONE_FRAC,
) -> dict:
    """Poll frame_source() until a confident soccer_ball detection appears,
    fire exactly one bounded robot.drive() call toward it, and return.

    frame_source is a zero-arg callable returning the latest frame_bgr or
    None - real use wraps a cv2.VideoCapture (see camera_frame_source()
    below), the self-test below wraps a fake with no camera at all.

    Call this the instant the challenge starts (i.e. right when
    run_program() enables the routine - see robot_client.py) so `start`
    below truly marks "challenge start", matching how every other routine
    in this repo is invoked.
    """
    start = time.monotonic()
    print("[FASTEST-BALL] challenge started - polling for a confident soccer_ball detection")

    while True:
        if (time.monotonic() - start) >= max_seconds:
            elapsed = time.monotonic() - start
            print(f"[FASTEST-BALL] TIMEOUT after {elapsed:.3f}s - no confident detection, NOT driving blind")
            return {"elapsed_seconds": None, "action": None, "confidence": None}

        frame = frame_source()
        if frame is None:
            time.sleep(POLL_SLEEP_SEC)  # see POLL_SLEEP_SEC comment above - small deliberate yield
            continue

        detections = detector.infer(frame)
        ball = detector.best(detections, "soccer_ball")
        if ball is None or ball.confidence < min_confidence:
            continue  # not confident enough yet - keep polling, do not sleep on a real frame miss

        frame_w = frame.shape[1]
        offset = ball.center_x - frame_w / 2.0
        if abs(offset) > deadzone_frac * frame_w:
            action = "rotate_left" if offset < 0 else "rotate_right"
            ms = TURN_MS
        else:
            action = "forward"
            ms = DRIVE_MS

        # Snapshot elapsed time HERE, at the instant we decide to move -
        # this is what "time until the robot initiates movement" actually
        # means. Measuring after robot.drive() returns would fold in
        # drive()'s post-move settle sleep (robot_client.py), which happens
        # after the motors are already commanded and so is not part of what
        # this challenge judges.
        elapsed = time.monotonic() - start
        print(
            f"[FASTEST-BALL] ball detected (confidence={ball.confidence:.2f}, offset={offset:.0f}px) "
            f"-> {action} | ELAPSED = {elapsed:.3f}s"
        )
        robot.drive(action, speed=APPROACH_SPEED, ms=ms)
        return {"elapsed_seconds": elapsed, "action": action, "confidence": ball.confidence}


try:
    import cv2  # only needed by camera_frame_source() below - guarded so the rest of this
except ImportError:  # module stays importable/py_compile-able with nothing but the stdlib.
    cv2 = None


def camera_frame_source(cap) -> Callable[[], Optional[Any]]:
    """Real-camera adapter for later on-device integration - NOT used by the
    self-test below. Wrap a cv2.VideoCapture in this to get a frame_source
    callable for run_fastest_ball_detection(). Calls cap.read() exactly once
    per call, matching cv2's (ok, frame) return contract - unusable without
    cv2 installed, which is fine, nothing else in this module needs cv2."""

    def _read() -> Optional[Any]:
        if cv2 is None:
            raise RuntimeError("cv2 is not installed - camera_frame_source() is unusable")
        ok, frame = cap.read()
        return frame if ok else None

    return _read


if __name__ == "__main__":
    # Camera-free, robot-free, model-free self-test - mirrors soccer_policy.py's
    # fake-object style exactly, so this runs tonight on a plain laptop.

    class _FakeFrame:
        def __init__(self, height: int, width: int) -> None:
            self.shape = (height, width, 3)

    class _FakeDetection:
        def __init__(self, label: str, confidence: float, x: int, y: int, width: int, height: int) -> None:
            self.label = label
            self.confidence = confidence
            self.x = x
            self.y = y
            self.width = width
            self.height = height

        @property
        def center_x(self) -> float:
            return self.x + self.width / 2

        @property
        def center_y(self) -> float:
            return self.y + self.height / 2

    class _FakeDetector:
        def __init__(self, detections) -> None:
            self.detections = detections

        def infer(self, frame_bgr):
            return self.detections

        def best(self, detections, label):
            matches = [d for d in detections if d.label == label]
            return max(matches, key=lambda d: d.confidence) if matches else None

    class _FakeRobot:
        def __init__(self) -> None:
            self.calls = []

        def drive(self, command, speed=150, ms=500):
            self.calls.append(("drive", command, speed, ms))

        def stop(self):
            self.calls.append(("stop",))

    class _FakeFrameSource:
        """Returns None for the first `empty_polls` calls (simulating 'no ball
        visible yet'), then returns `frame` on every call after that."""

        def __init__(self, empty_polls: int, frame) -> None:
            self.empty_polls = empty_polls
            self.frame = frame
            self.calls = 0

        def __call__(self):
            self.calls += 1
            return None if self.calls <= self.empty_polls else self.frame

    FRAME = _FakeFrame(240, 320)  # centre x = 160
    # Well left of centre (centre_x=10) so the deadzone branch is unambiguous.
    off_center_ball = _FakeDetection("soccer_ball", 0.9, x=0, y=150, width=20, height=20)

    print("[SELF-TEST] ball appears after a few empty polls -> exactly one drive(), sane bounded result")
    robot = _FakeRobot()
    detector = _FakeDetector([off_center_ball])
    frame_source = _FakeFrameSource(empty_polls=5, frame=FRAME)
    result = run_fastest_ball_detection(robot, detector, frame_source, max_seconds=10.0)
    print(f"  -> {result}")
    drive_calls = [c for c in robot.calls if c[0] == "drive"]
    assert len(drive_calls) == 1, f"expected exactly one drive() call, got {robot.calls}"
    assert isinstance(result["elapsed_seconds"], float), result
    # Not asserting a precise value (flaky) - just positive and comfortably bounded
    # given the fake's tiny simulated (5 * POLL_SLEEP_SEC) delay.
    assert 0.0 < result["elapsed_seconds"] < 2.0, result
    assert result["action"] in ("rotate_left", "rotate_right", "forward"), result
    assert result["action"] == "rotate_left", f"ball is left of centre, expected rotate_left, got {result}"
    assert result["confidence"] == 0.9, result
    print(f"  -> exactly 1 drive() call, elapsed={result['elapsed_seconds']:.3f}s, action={result['action']}")

    print("[SELF-TEST] timeout path: frame_source always returns None -> no drive(), all-None result")
    robot = _FakeRobot()
    detector = _FakeDetector([])
    result = run_fastest_ball_detection(robot, detector, lambda: None, max_seconds=0.2)
    print(f"  -> {result}")
    assert result == {"elapsed_seconds": None, "action": None, "confidence": None}, result
    assert robot.calls == [], f"must never drive blind on timeout, got {robot.calls}"

    print("SELF-TEST PASSED")
