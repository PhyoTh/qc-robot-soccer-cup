"""
sim_match.py  -  multi-tick integration storyboard for SoccerPolicy.

python/soccer_policy.py already self-tests every decision branch in
isolation (own-goal avoidance, ball-chase, search, foul-avoidance, the
hybrid tracker's tracking/coasting/lost states, possession-safe scanning) -
each scenario builds a *fresh* SoccerPolicy(). That catches branch-level bugs
but can't catch bugs that only show up across a sequence of ticks on ONE
continuing policy instance: state that should persist (or shouldn't) between
calls, and whether the priority ordering (opponent-contact > ball-chase >
search) holds up when situations change tick to tick, the way they actually
will during a live 5-minute match.

This file runs a single SoccerPolicy through a scripted ~13-tick "story":
search (biased, alternating) -> find the ball -> chase it -> get cut off by
an opponent at EMERGENCY range (retreat) -> find the ball again but no goal
is readable yet (possession-safe peek, not a blind push) -> get faked out by
our own goal -> score on the opponent's goal -> get contested by an opponent
at ordinary range (juke sideways, NOT retreat - this is the exploit fix) ->
lose the ball long enough to exhaust the coast window -> search resumes,
biased toward the side the ball was last really seen on.

Deliberately does NOT re-derive the hybrid tracker's exact EMA/coast-window
float math here - soccer_policy.py's own self-test already covers that with
clean, controlled inputs. This file only asserts the FINAL search direction
after the coast window expires (a state-persistence check), not the
intermediate coasting ticks' exact values.

Uses the same lightweight fakes as soccer_policy.py's own self-test (no cv2,
numpy, or real robot needed) so it runs anywhere python3 runs.
"""
from __future__ import annotations

from soccer_policy import (
    SoccerPolicy,
    APPROACH_SPEED,
    SEARCH_SPEED,
    TURN_MS,
    DRIVE_MS,
    MIN_APPROACH_SPEED,
    POSSESSION_SCAN_SPEED,
    POSSESSION_SCAN_TURN_MS,
    COAST_TICKS,
)


class _FakeFrame:
    def __init__(self, height: int = 240, width: int = 320) -> None:
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


class _ScriptedDetector:
    """Each call to infer() returns the next scripted detection list,
    regardless of the frame passed in - the story controls what's "seen"."""

    def __init__(self, script: list[list[_FakeDetection]]) -> None:
        self._script = list(script)

    def infer(self, frame_bgr):
        if not self._script:
            raise AssertionError("story ran out of scripted ticks before the policy did")
        return self._script.pop(0)

    def best(self, detections, label):
        matches = [d for d in detections if d.label == label]
        return max(matches, key=lambda d: d.confidence) if matches else None


class _ScriptedWallDetector:
    """Each call to classify() returns the next scripted result - lets the
    story control "what wall color is visible" independently of the
    detector's ball/robot/goal boxes, exactly like a real camera frame would
    show both at once."""

    def __init__(self, script: list[str]) -> None:
        self._script = list(script)

    def classify(self, frame_bgr, team_is_blue: bool) -> dict:
        result = self._script.pop(0) if self._script else "UNKNOWN"
        wall = "RED" if result == "OWN SIDE" else "BLUE" if result == "OPPONENT SIDE" else "UNKNOWN"
        return {"side": wall, "red_pct": 0.0, "blue_pct": 0.0, "team": "RED", "wall": wall, "result": result}

    def field_line(self, classification: dict) -> str:
        return f"[FIELD] (sim) -> {classification['result']}"


class _RecordingRobot:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def drive(self, command, speed=150, ms=500):
        self.calls.append(("drive", command, speed, ms))

    def stop(self):
        self.calls.append(("stop",))
        return True


FRAME = _FakeFrame()  # 320-wide -> centre x = 160
SENSORS_CLEAR = {"program_enabled": True, "ultrasonic_mm": -1}


def run_story() -> None:
    ball_far_left = _FakeDetection("soccer_ball", 0.9, x=0, y=150, width=15, height=15)         # centre_x=7.5
    ball_centered_far = _FakeDetection("soccer_ball", 0.9, x=150, y=150, width=15, height=15)    # centre_x=157.5, far
    ball_centered_close = _FakeDetection("soccer_ball", 0.9, x=140, y=150, width=60, height=60)  # centre_x=170, close

    # Emergency: fills most of the frame, ultrasonic well inside COLLISION_IMMINENT_MM (100).
    opponent_emergency = _FakeDetection("robot", 0.9, x=20, y=20, width=250, height=200)  # width_frac=0.78

    # Contested: noticeably present but not overwhelming, ultrasonic inside CONTESTED_MM (200) but
    # outside COLLISION_IMMINENT_MM (100). Positioned right-of-centre so the expected juke is LEFT
    # (away from the opponent), demonstrating this doesn't just retreat like the emergency case does.
    opponent_contested = _FakeDetection("robot", 0.9, x=170, y=20, width=130, height=100)  # width_frac=0.41, centre_x=235

    own_goal_box = _FakeDetection("goal", 0.9, x=100, y=20, width=120, height=60)
    opp_goal_box = _FakeDetection("goal", 0.9, x=100, y=20, width=120, height=60)

    detector_script = [
        [],                                            # 1: nothing yet
        [],                                             # 2: still nothing
        [ball_far_left],                                 # 3: ball appears, off-centre
        [ball_far_left, opponent_emergency],              # 4: opponent right on top of us - EMERGENCY
        [ball_centered_far],                              # 5: ball re-found and centred, but the tape is unreadable
        [ball_centered_close, own_goal_box],               # 6: closing in, and the end ahead is OURS
        [ball_centered_close, opp_goal_box],               # 7: same spot, end ahead is the OPPONENT'S - score
        [ball_centered_close, opponent_contested],          # 8: opponent contests at ORDINARY range - juke, don't retreat
    ] + [[]] * (COAST_TICKS + 1)  # 9..12+: ball gone long enough to exhaust the coast window -> search

    # classify() is called on every tick that REACHES the scoring decision -
    # i.e. ball tracking AND centred - regardless of whether a goal was
    # detected (soccer_policy design note 7). That is ticks 5, 6 and 7 here;
    # ticks 1-4 return earlier (searching / aiming / opponent emergency) and
    # ticks 8+ do too (opponent contest, then ball lost), so they never
    # consume from this list.
    wall_script = [
        "UNKNOWN",         # tick 5: tape not readable from here -> possession-safe peek
        "OWN SIDE",        # tick 6: end ahead is ours -> must not push
        "OPPONENT SIDE",   # tick 7: end ahead is theirs -> push through
    ]

    robot = _RecordingRobot()
    policy = SoccerPolicy(robot)
    detector = _ScriptedDetector(detector_script)
    wall = _ScriptedWallDetector(wall_script)

    def tick(sensors=SENSORS_CLEAR):
        policy.decide_and_act(FRAME, sensors, detector, wall, hold_toggle=False)
        return robot.calls[-1]

    print("[SIM] tick 1: no ball -> search, arbitrary first guess")
    first_search = tick()
    assert first_search[0] == "drive" and first_search[1] in ("rotate_left", "rotate_right"), first_search

    print("[SIM] tick 2: still no ball -> search the OTHER direction (alternation)")
    second_search = tick()
    assert second_search[1] != first_search[1], (first_search, second_search)

    print("[SIM] tick 3: ball far left -> turn toward it")
    assert tick() == ("drive", "rotate_left", APPROACH_SPEED, TURN_MS)

    print("[SIM] tick 4: opponent EMERGENCY-close -> back off, ball is ignored entirely")
    assert tick({"program_enabled": True, "ultrasonic_mm": 80}) == ("drive", "backward", APPROACH_SPEED, DRIVE_MS)

    print("[SIM] tick 5: ball re-found and centred, but the tape is UNREADABLE -> possession-safe peek, not a blind push")
    action = tick()
    assert action[0] == "drive" and action[1] in ("left", "right"), action
    assert action[2:] == (POSSESSION_SCAN_SPEED, POSSESSION_SCAN_TURN_MS), (
        "must be the gentle possession-safe peek, not a full push or a full search", action
    )

    print("[SIM] tick 6: ball centred + close + OUR end ahead -> peel off, never forward")
    action = tick()
    assert action == ("drive", "right", SEARCH_SPEED, TURN_MS), action

    print("[SIM] tick 7: ball centred + close + OPPONENT end ahead -> push through")
    action = tick()
    expected_speed = max(int(APPROACH_SPEED * (1.0 - ball_centered_close.width / FRAME.shape[1])), MIN_APPROACH_SPEED)
    assert action == ("drive", "forward", expected_speed, DRIVE_MS), action

    print("[SIM] tick 8: opponent contests at ORDINARY range -> juke sideways, must NOT retreat (the exploit fix)")
    action = tick({"program_enabled": True, "ultrasonic_mm": 150})  # inside CONTESTED_MM, outside COLLISION_IMMINENT_MM
    assert action[1] != "backward", f"ordinary contested proximity must not force a retreat, got {action}"
    assert action == ("drive", "left", APPROACH_SPEED, TURN_MS), (
        "opponent was right-of-centre - expected a juke to the LEFT (away from them), got", action
    )

    print(f"[SIM] ticks 9..{9 + COAST_TICKS}: ball gone long enough to exhaust the coast window -> search resumes")
    for _ in range(COAST_TICKS + 1):
        final_search = tick()
    assert final_search[0] == "drive" and final_search[1] in ("rotate_left", "rotate_right"), final_search
    assert final_search[2:] == (SEARCH_SPEED, TURN_MS), (
        "expected a genuine search action once the coast window is exhausted", final_search
    )
    # The ball was last really seen at centre_x=170 (slightly RIGHT of the 160 frame centre, ticks
    # 6-8) - the search should be biased to look right first, not a context-free coin flip.
    assert final_search[1] == "rotate_right", (
        "search should be biased toward the side the ball was last actually seen on (right of centre)",
        final_search,
    )

    print(f"[SIM] full action sequence: {[c[1] for c in robot.calls]}")
    print("SIM-MATCH STORY PASSED")


if __name__ == "__main__":
    run_story()
