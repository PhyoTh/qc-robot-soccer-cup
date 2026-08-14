"""
Redemption Cup -> Best Goal Celebration.

Judged on creativity, and the read here is deliberate: CUTE over "cool" -
most crowds warm up to a shy, bashful little robot more than a confident
victory spin. That shows up as small wiggles instead of a full-speed spin,
a giggly uneven buzzer rhythm instead of evenly-spaced chirps, and a soft
pastel color palette instead of saturated party-light colors. Call
celebrate() right after the match logic registers a scored goal.
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_client import MiniAutoRobot

# --- Tunables: the "cute, not cool" choreography -----------------------

# A shy beat before showing off - "did I just do that?"
BASHFUL_PAUSE_SECONDS = 0.35

# Small alternating rotate pulses - a shiver of excitement, not a spin.
# Deliberately gentler/shorter than a normal turn (compare soccer_policy.py's
# TURN_MS=200 at APPROACH_SPEED=150) - this IS the "cute" choice, not a bug.
WIGGLE_COUNT = 4
WIGGLE_SPEED = 140
WIGGLE_MS = 120
WIGGLE_PAUSE_SECONDS = 0.08

# A tiny forward-then-back "hop in place" after the wiggle.
BOUNCE_SPEED = 130
BOUNCE_MS = 100
BOUNCE_PAUSE_SECONDS = 0.1

# Giggle rhythm: uneven spacing (quick-quick...pause...one more) instead of
# evenly-spaced chirps - reads as a giggle, not a siren. Each value is the
# pause BEFORE that buzz() call.
GIGGLE_PATTERN = [0.0, 0.15, 0.45]

# Soft pastel cycle, only used when robot.rgb() is available on-device.
# Deliberately lower-saturation than a "party" palette (compare a full
# (255, 0, 0) red) - this single choice does most of the "cute vs cool" work.
BLUSH_COLORS = [
    (255, 209, 220),   # baby pink
    (255, 179, 199),   # a touch deeper - a little "blush"
    (255, 209, 220),   # back to baby pink - a soft pulse, not a rainbow sweep
    (255, 241, 224),   # warm cream
    (224, 247, 250),   # powder sky blue
    (230, 220, 250),   # soft lavender
]
COLOR_STEP_SECONDS = 0.18

# Gentle led() fallback when rgb() isn't available - a soft blink, not a strobe.
LED_FALLBACK_BLINKS = 3
LED_BLINK_SECONDS = 0.25


def celebrate(robot: MiniAutoRobot, cycles: int = 2) -> None:
    """Bashful pause + small wiggle + happy hop + giggly chirps + pastel
    color blush. Run after a goal.

    Uses robot.rgb() when the team has landed that method and the firmware
    has been reflashed; otherwise falls back to a gentle led() blink.
    ProgramStopped (BOOT button disabled the program mid-move) is allowed to
    propagate so the caller's run_program() can handle it the normal way.
    """
    try:
        rgb = getattr(robot, "rgb", None)

        for cycle in range(cycles):
            print(f"[TEAM] cute goal celebration cycle {cycle + 1}/{cycles}")

            time.sleep(BASHFUL_PAUSE_SECONDS)

            wiggle_left = True
            for _ in range(WIGGLE_COUNT):
                direction = "rotate_left" if wiggle_left else "rotate_right"
                robot.drive(direction, speed=WIGGLE_SPEED, ms=WIGGLE_MS)
                time.sleep(WIGGLE_PAUSE_SECONDS)
                wiggle_left = not wiggle_left

            robot.drive("forward", speed=BOUNCE_SPEED, ms=BOUNCE_MS)
            time.sleep(BOUNCE_PAUSE_SECONDS)
            robot.drive("backward", speed=BOUNCE_SPEED, ms=BOUNCE_MS)

            for delay in GIGGLE_PATTERN:
                time.sleep(delay)
                robot.buzz()

            if rgb is not None:
                for r, g, b in BLUSH_COLORS:
                    rgb(r, g, b)
                    time.sleep(COLOR_STEP_SECONDS)
            else:
                for _ in range(LED_FALLBACK_BLINKS):
                    robot.led(True)
                    time.sleep(LED_BLINK_SECONDS)
                    robot.led(False)
                    time.sleep(LED_BLINK_SECONDS)
    finally:
        robot.stop()


if __name__ == "__main__":
    # Camera-free, robot-free self-test: a fake robot records every call so we
    # can assert on the exact sequence, instead of only checking that this
    # module imports without crashing.

    class _ProgramStopped(Exception):
        """Stand-in for robot_client.ProgramStopped - celebrate() doesn't care
        about the concrete type, it just lets whatever the robot raises
        propagate through its bare try/finally, so any Exception subclass
        exercises the same code path as the real one."""

    class _FakeRobot:
        def __init__(self, has_rgb: bool, fail_on_call: int = None) -> None:
            self.calls = []
            self._fail_on_call = fail_on_call
            self._call_count = 0
            if has_rgb:
                # Only attach the attribute when "the firmware has been
                # reflashed with rgb()" - mirrors getattr(robot, "rgb", None).
                self.rgb = self._rgb

        def _record(self, *entry) -> None:
            self._call_count += 1
            self.calls.append(entry)
            if self._fail_on_call is not None and self._call_count == self._fail_on_call:
                raise _ProgramStopped("BOOT button disabled mid-move")

        def drive(self, command, speed=150, ms=500):
            self._record("drive", command, speed, ms)

        def buzz(self):
            self._record("buzz")
            return True

        def led(self, on):
            self._record("led", on)
            return True

        def _rgb(self, r, g, b):
            self._record("rgb", r, g, b)
            return True

        def stop(self):
            self.calls.append(("stop",))
            return True

    DRIVES_PER_CYCLE = WIGGLE_COUNT + 2  # the wiggle pulses + the forward/backward hop

    print("[SELF-TEST] rgb() available -> pastel blush cycle, never falls back to led() blink")
    robot = _FakeRobot(has_rgb=True)
    celebrate(robot, cycles=1)
    rgb_calls = [c for c in robot.calls if c[0] == "rgb"]
    led_calls = [c for c in robot.calls if c[0] == "led"]
    assert len(rgb_calls) == len(BLUSH_COLORS), rgb_calls
    assert [c[1:] for c in rgb_calls] == BLUSH_COLORS, "expected the pastel palette in order, not a party-color cycle"
    assert led_calls == [], "must not blink led() when rgb() exists"
    assert robot.calls[-1] == ("stop",), "must always stop() at the end"
    print(f"  -> {len(rgb_calls)} pastel rgb() steps, ended with stop()")

    print("[SELF-TEST] rgb() missing -> falls back to led() blinking")
    robot = _FakeRobot(has_rgb=False)
    celebrate(robot, cycles=1)
    rgb_calls = [c for c in robot.calls if c[0] == "rgb"]
    led_calls = [c for c in robot.calls if c[0] == "led"]
    assert rgb_calls == [], "must not call rgb() when the attribute doesn't exist"
    assert len(led_calls) == LED_FALLBACK_BLINKS * 2, led_calls  # True/False pair per blink
    print(f"  -> {len(led_calls)} led() blink calls (no rgb available)")

    print("[SELF-TEST] wiggle alternates direction, every drive() pulse stays small and bounded")
    robot = _FakeRobot(has_rgb=True)
    celebrate(robot, cycles=1)
    drive_calls = [c for c in robot.calls if c[0] == "drive"]
    assert len(drive_calls) == DRIVES_PER_CYCLE, drive_calls
    wiggle_calls = drive_calls[:WIGGLE_COUNT]
    wiggle_dirs = [c[1] for c in wiggle_calls]
    assert wiggle_dirs == ["rotate_left", "rotate_right", "rotate_left", "rotate_right"], wiggle_dirs
    assert all(c[3] <= 500 for c in drive_calls), "every drive() pulse must stay bounded (<=500ms)"
    assert all(c[3] <= WIGGLE_MS + 1 for c in wiggle_calls), "wiggle pulses should be gentle/short, not a full spin"
    hop_calls = drive_calls[WIGGLE_COUNT:]
    assert [c[1] for c in hop_calls] == ["forward", "backward"], hop_calls
    print(f"  -> wiggle={wiggle_dirs}  hop={[c[1] for c in hop_calls]}")

    print("[SELF-TEST] giggle rhythm: 3 chirps with uneven spacing, not evenly-spaced")
    robot = _FakeRobot(has_rgb=True)
    celebrate(robot, cycles=2)
    buzz_calls = [c for c in robot.calls if c[0] == "buzz"]
    assert len(buzz_calls) == len(GIGGLE_PATTERN) * 2, buzz_calls
    assert len(set(GIGGLE_PATTERN)) > 1, "the giggle pattern should be uneven, not a uniform beat"
    print(f"  -> {len(buzz_calls)} buzzer chirps across 2 cycles, pattern={GIGGLE_PATTERN}")

    print("[SELF-TEST] robot.stop() still runs even if a move raises mid-routine")
    robot = _FakeRobot(has_rgb=True, fail_on_call=2)  # fail on the 2nd recorded call (2nd wiggle pulse)
    try:
        celebrate(robot, cycles=1)
        raise AssertionError("expected _ProgramStopped to propagate out of celebrate()")
    except _ProgramStopped:
        pass
    assert robot.calls[-1] == ("stop",), "finally: robot.stop() must still run after an exception"
    print("  -> stop() ran even though a move raised mid-routine, and the exception still propagated")

    print("SELF-TEST PASSED")
