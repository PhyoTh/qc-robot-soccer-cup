"""
trick_shot.py  -  Redemption Cup -> Trick Shot Challenge decision loop.

Rule reminder (Redemption Cup, Trick Shot Challenge): the robot must score a
goal while navigating around obstacles placed on the course. Unlike the main
Robot Soccer Cup match, this event adds obstacles that must be dodged, not
pushed through.

This module deliberately does NOT reimplement ball-chase / goal-scoring /
own-goal-avoidance / opponent-foul-avoidance - soccer_policy.SoccerPolicy
already does all of that correctly and is separately self-tested. Instead,
TrickShotPolicy COMPOSES a real SoccerPolicy instance and adds exactly one
new capability in front of it: obstacle dodging via the ultrasonic sensor,
because the FOMO object-detection model only knows "goal" / "robot" /
"soccer_ball" - it has no "obstacle" class, so ultrasonic is the only sensor
that can see a generic obstacle at all.

THE HARD PART, FLAGGED PLAINLY: a close ultrasonic reading is ambiguous. It
fires both when a real obstacle is in the way (dodge!) and when the robot is
simply right up against the ball about to push it (normal, desired, do NOT
dodge). This file disambiguates the two cases by re-checking, on the SAME
sensor read, whether the soccer_ball is detected, centred, and large in
frame (i.e. genuinely close and directly ahead) - if so the close reading is
"explained" by the ball and is not treated as an obstacle. OBSTACLE_MM and
the "ball explains it" centering/size thresholds below are judgment calls
made tonight with no real ultrasonic/camera data on the actual course - the
team should watch this closely during tomorrow's on-hardware testing and
retune these constants against the real obstacles and real ball approach
distances before relying on them in the run.

Only project-local modules are imported here (soccer_policy directly - it
has no hard dependency on cv2/numpy/edge_impulse_linux/arduino.app_utils
either - plus ei_runner/robot_client/wall_detector under TYPE_CHECKING for
type hints only), so this module stays importable and py_compile-able on a
plain laptop with none of those packages installed.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from soccer_policy import SoccerPolicy

if TYPE_CHECKING:
    from ei_runner import ObjectDetector
    from robot_client import MiniAutoRobot
    from wall_detector import WallSideDetector

# --- Tunables: retune here once real hardware/course data is available ----

# Ultrasonic reading below which we treat a reading as "something is close"
# at all. Tighter than soccer_policy's OPPONENT_BACKOFF_MM (150mm) would be
# wrong here - OPPONENT_BACKOFF_MM is about not fouling another robot, this
# is about not driving into a static obstacle - but the Trick Shot course is
# EXPECTED to have obstacles nearby throughout, so we deliberately react a
# bit earlier than that. UNVALIDATED against real hardware - see module
# docstring.
OBSTACLE_MM = 200

# "Ball explains the close reading" thresholds - reuses the same kind of
# centering check as soccer_policy.CENTER_DEADZONE_FRAC, plus a size check
# that soccer_policy doesn't need (it uses ball size only to slow the
# approach, never to decide "is this really close").
BALL_EXPLAINS_CENTER_FRAC = 0.15  # ball centre must be within this frac of frame width from centre
BALL_EXPLAINS_MIN_WIDTH_FRAC = 0.35  # ball bbox must fill at least this much of the frame width

DODGE_SPEED = 130  # between soccer_policy's SEARCH_SPEED and APPROACH_SPEED
DODGE_MS = 220      # bounded strafe pulse, same order of magnitude as soccer_policy's TURN_MS/DRIVE_MS


class TrickShotPolicy:
    """Wraps a real SoccerPolicy and intercepts only the obstacle-dodge
    decision; every other tick is delegated to the inner policy unchanged."""

    def __init__(self, robot: "MiniAutoRobot") -> None:
        self.robot = robot
        self._policy = SoccerPolicy(robot)
        self._dodge_left_next = True  # alternates left/right dodge, same idea as SoccerPolicy's search alternation

    def reset(self) -> None:
        """Passthrough to the wrapped SoccerPolicy's reset() - see its
        docstring. Call this at the start of every fresh program-enabled
        session for the same reason: stale ball-tracking/search state from
        before a pause or reposition shouldn't carry into a new one."""
        self._policy.reset()
        self._dodge_left_next = True

    def decide_and_act(
        self,
        frame_bgr,
        sensors: dict,
        detector: "ObjectDetector",
        wall: "WallSideDetector",
        hold_toggle: bool,
        frame_ts: Optional[float] = None,
    ) -> None:
        """Run one Decide -> Act tick. Checks for a real obstacle first; if
        found, dodges (one bounded strafe) and returns without touching the
        inner policy this tick. Otherwise delegates the entire tick to
        self._policy.decide_and_act() unchanged."""

        # Basic guards (disabled program, missing/stale frame, empty frame)
        # are already handled correctly by SoccerPolicy.decide_and_act() -
        # duplicating that logic here would just be a second place for it to
        # go stale. We only need to intercept once frame_bgr/sensors are
        # known-good, which requires running our own obstacle check below;
        # any case that check can't safely run in, we just hand straight to
        # the inner policy and let it apply those same guards.
        if not sensors.get("program_enabled") or frame_bgr is None:
            self._policy.decide_and_act(frame_bgr, sensors, detector, wall, hold_toggle, frame_ts)
            return

        frame_h, frame_w = frame_bgr.shape[0], frame_bgr.shape[1]
        if frame_w <= 0 or frame_h <= 0:
            self._policy.decide_and_act(frame_bgr, sensors, detector, wall, hold_toggle, frame_ts)
            return

        ultrasonic_mm = sensors.get("ultrasonic_mm", -1)
        if ultrasonic_mm > 0 and ultrasonic_mm < OBSTACLE_MM:
            # Something is close. Run our own inference pass to check whether
            # the ball explains it - yes, this means detector.infer() can run
            # twice this tick (once here, once inside the inner policy if we
            # end up delegating anyway). The model is ~5ms per the team's
            # Edge Impulse dashboard, so paying that twice is a deliberate
            # simplicity-over-micro-optimization tradeoff, not an oversight.
            try:
                detections = detector.infer(frame_bgr)
            except Exception as exc:  # noqa: BLE001 - one bad frame must not crash the match loop
                print(f"[WARN] trick_shot: inference failed ({exc}) - stopping")
                self.robot.stop()
                return

            ball = detector.best(detections, "soccer_ball")
            ball_explains_it = False
            if ball is not None:
                offset = ball.center_x - frame_w / 2.0
                centered = abs(offset) <= BALL_EXPLAINS_CENTER_FRAC * frame_w
                large = (ball.width / frame_w) >= BALL_EXPLAINS_MIN_WIDTH_FRAC
                ball_explains_it = centered and large

            if not ball_explains_it:
                direction = "left" if self._dodge_left_next else "right"
                self._dodge_left_next = not self._dodge_left_next
                print(
                    f"[TRICK-SHOT] obstacle at ultrasonic_mm={ultrasonic_mm} "
                    f"(ball_explains_it={ball_explains_it}) - dodging {direction}"
                )
                self.robot.drive(direction, speed=DODGE_SPEED, ms=DODGE_MS)
                return

            print(
                f"[TRICK-SHOT] close ultrasonic_mm={ultrasonic_mm} explained by centred/large "
                "ball - not an obstacle, delegating to inner policy"
            )

        # No obstacle (or the ball explains the close reading): the entire
        # rest of the decision - ball chase, search, own-goal-avoidance,
        # opponent-foul-avoidance - is SoccerPolicy's job, reused as-is.
        self._policy.decide_and_act(frame_bgr, sensors, detector, wall, hold_toggle, frame_ts)


if __name__ == "__main__":
    # Camera-free, robot-free, model-free self-test mirroring soccer_policy.py's
    # fake-object style exactly, so it runs tonight on a plain laptop with
    # nothing attached.

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
            self.infer_calls = 0

        def infer(self, frame_bgr):
            self.infer_calls += 1
            return self.detections

        def best(self, detections, label):
            matches = [d for d in detections if d.label == label]
            return max(matches, key=lambda d: d.confidence) if matches else None

    class _FakeWallDetector:
        """Goal-side classification is already covered by soccer_policy.py's
        own self-tests - this file only needs a fixed stand-in, mostly
        UNKNOWN, plus one OPPONENT-SIDE variant for the delegation test
        below (which needs the inner policy to resolve to a clean push)."""

        def __init__(self, result: str = "UNKNOWN") -> None:
            self._result = result

        def classify(self, frame_bgr, team_is_blue: bool) -> dict:
            return {"side": self._result, "red_pct": 0.0, "blue_pct": 0.0, "team": "UNKNOWN", "wall": self._result, "result": self._result}

        def field_line(self, classification: dict) -> str:
            return f"[FIELD] (self-test) -> {classification['result']}"

    class _FakeRobot:
        def __init__(self) -> None:
            self.calls = []

        def drive(self, command, speed=150, ms=500):
            self.calls.append(("drive", command, speed, ms))

        def stop(self):
            self.calls.append(("stop",))
            return True

    FRAME = _FakeFrame(240, 320)  # centre x = 160
    WALL = _FakeWallDetector("UNKNOWN")
    WALL_OPPONENT = _FakeWallDetector("OPPONENT SIDE")

    print("[SELF-TEST] obstacle ahead, ball not explaining it -> dodges instead of driving into it")
    robot = _FakeRobot()
    policy = TrickShotPolicy(robot)
    sensors = {"program_enabled": True, "ultrasonic_mm": 120}  # < OBSTACLE_MM
    detector = _FakeDetector([])  # no ball at all -> cannot explain the close reading
    policy.decide_and_act(FRAME, sensors, detector, WALL, hold_toggle=False)
    assert robot.calls, "expected an action"
    last = robot.calls[-1]
    assert last[0] == "drive" and last[1] in ("left", "right"), f"expected a dodge strafe, got {last}"
    assert last == ("drive", "left", DODGE_SPEED, DODGE_MS), f"expected the dodge pulse, got {last}"
    print(f"  -> {last}  (correctly dodged instead of driving into the obstacle)")

    print("[SELF-TEST] obstacle dodge direction alternates across calls")
    sensors2 = {"program_enabled": True, "ultrasonic_mm": 90}
    detector2 = _FakeDetector([])
    policy.decide_and_act(FRAME, sensors2, detector2, WALL, hold_toggle=False)
    first_dir, second_dir = robot.calls[0][1], robot.calls[1][1]
    assert first_dir != second_dir, robot.calls
    print(f"  -> {first_dir} then {second_dir}  (alternates so it doesn't dodge the same way forever)")

    print("[SELF-TEST] ball centred+large + close ultrasonic -> delegates to inner policy's forward push, no dodge")
    robot = _FakeRobot()
    policy = TrickShotPolicy(robot)
    close_ball = _FakeDetection("soccer_ball", 0.9, x=90, y=150, width=140, height=140)  # centre_x=160 (frame centre), width_frac=0.44
    opp_goal = _FakeDetection("goal", 0.9, x=100, y=20, width=120, height=60)
    sensors = {"program_enabled": True, "ultrasonic_mm": 100}  # < OBSTACLE_MM, would be an obstacle if unexplained
    # A goal is included and WALL_OPPONENT confirms it's the opponent's, so the
    # inner SoccerPolicy's own-goal-avoidance/possession-safe-scan logic (see
    # soccer_policy.py) resolves cleanly to a forward push - this test is about
    # TrickShotPolicy correctly declining to dodge, not about re-proving
    # SoccerPolicy's own goal-side decision, which has its own dedicated tests.
    detector = _FakeDetector([close_ball, opp_goal])
    policy.decide_and_act(FRAME, sensors, detector, WALL_OPPONENT, hold_toggle=False)
    last = robot.calls[-1]
    assert last[0] == "drive" and last[1] != "left" and last[1] != "right", (
        f"expected the inner policy to act (a forward push), not a dodge, got {last}"
    )
    assert last[2:] != (DODGE_SPEED, DODGE_MS), f"must not be a dodge pulse, got {last}"
    print(f"  -> {last}  (correctly recognised the ball explains the close reading, did not dodge)")
    assert detector.infer_calls == 2, (
        f"expected 2 infer() calls this tick (our own check + the delegated inner-policy call), "
        f"got {detector.infer_calls}"
    )
    print(f"  -> detector.infer() called {detector.infer_calls}x this tick (accepted double-inference tradeoff)")

    print("[SELF-TEST] no obstacle at all -> normal ball-chase behaviour happens via the inner policy")
    robot = _FakeRobot()
    policy = TrickShotPolicy(robot)
    left_ball = _FakeDetection("soccer_ball", 0.9, x=0, y=150, width=20, height=20)  # centre_x=10, off-centre left
    sensors = {"program_enabled": True, "ultrasonic_mm": -1}  # no valid reading -> never an obstacle
    detector = _FakeDetector([left_ball])
    policy.decide_and_act(FRAME, sensors, detector, WALL, hold_toggle=False)
    last = robot.calls[-1]
    assert last[0] == "drive" and last[1] == "rotate_left", f"expected the inner policy's aiming turn, got {last}"
    print(f"  -> {last}  (delegated straight to the inner policy's ball-chase)")
    assert detector.infer_calls == 1, f"no obstacle check needed -> expected exactly 1 infer() call, got {detector.infer_calls}"

    print("[SELF-TEST] program disabled -> delegates to inner policy, no action at all")
    robot = _FakeRobot()
    policy = TrickShotPolicy(robot)
    disabled_sensors = {"program_enabled": False, "ultrasonic_mm": -1}
    policy.decide_and_act(FRAME, disabled_sensors, _FakeDetector([]), WALL, hold_toggle=False)
    assert robot.calls == [], robot.calls

    print("[SELF-TEST] missing frame -> delegates to inner policy, stop(), never drive blind")
    robot = _FakeRobot()
    policy = TrickShotPolicy(robot)
    enabled_sensors = {"program_enabled": True, "ultrasonic_mm": -1}
    policy.decide_and_act(None, enabled_sensors, _FakeDetector([]), WALL, hold_toggle=False)
    assert robot.calls == [("stop",)], robot.calls

    print("SELF-TEST PASSED")
