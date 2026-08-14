"""
Redemption Cup -> Precision Course.

Follow a taped course using the onboard 4-channel line sensor, judged on
speed and accuracy. Runs a bounded sense -> decide -> act -> reobserve loop:
one short robot.drive() correction per tick, then straight back to reading
sensors before the next action.
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_client import MiniAutoRobot

try:
    from robot_client import ProgramStopped
except ImportError:
    # robot_client.py does an unconditional `from arduino.app_utils import
    # Bridge` at module load, which only succeeds inside the UNO Q App Lab
    # runtime. On a plain laptop that import fails, so fall back to a local
    # stand-in - this file must still be *callable* (not just importable) for
    # its own self-test below. On-device, the `try` above always succeeds and
    # this fallback is never used, so real behavior is unaffected.
    class ProgramStopped(Exception):
        pass

# --- Tunables: retune here without touching the control loop below ---
LINE_WEIGHTS = [-3, -1, 1, 3]    # left-to-right, matches line_digital bit order

FORWARD_SPEED = 150
FORWARD_MS = 200                 # one drive() correction per tick, kept in the 150-250ms window

CORRECTION_SPEED = 150
CORRECTION_MS = 200

SEARCH_DIRECTION = "rotate_left"  # spin direction while hunting for a fully lost line
SEARCH_SPEED = 120
SEARCH_MS = 200

STOP_LINE_PAUSE_SECONDS = 0.3    # brief hold when all 4 sensors trip (intersection / stop-line)

MAX_SECONDS_DEFAULT = 60.0


def _steering_error(line_digital: list[int]) -> int:
    """Dot the 4 line bits against LINE_WEIGHTS; >0 means drift right, <0 means drift left."""
    return sum(bit * weight for bit, weight in zip(line_digital, LINE_WEIGHTS))


def run_precision_course(robot: MiniAutoRobot, max_seconds: float = MAX_SECONDS_DEFAULT) -> None:
    """Bounded line-follow loop for the Precision Course challenge.

    Every tick: read sensors, handle a bad read or the two all-bits special
    cases explicitly, otherwise steer off the weighted error - then loop back
    and reobserve before acting again. ProgramStopped (BOOT button disabled
    the program mid-move) is treated as a normal early exit, not a failure.
    """
    start = time.monotonic()
    try:
        while (time.monotonic() - start) < max_seconds and robot.is_running():
            sensors = robot.read_sensors()
            line_ok = bool(sensors.get("line_ok"))
            line_digital = sensors.get("line_digital") or [0, 0, 0, 0]

            if not line_ok:
                # Never trust a failed read - search instead of driving blind.
                print("[WARN] line sensor read invalid - searching")
                robot.drive(SEARCH_DIRECTION, speed=SEARCH_SPEED, ms=SEARCH_MS)
                continue

            if all(bit == 0 for bit in line_digital):
                print("[INFO] line lost (all-zero) - searching")
                robot.drive(SEARCH_DIRECTION, speed=SEARCH_SPEED, ms=SEARCH_MS)
                continue

            if all(bit == 1 for bit in line_digital):
                print("[INFO] intersection / stop-line detected (all-one) - holding")
                robot.stop()
                time.sleep(STOP_LINE_PAUSE_SECONDS)
                continue

            error = _steering_error(line_digital)
            if error > 0:
                print(f"[INFO] line={line_digital} error={error} - correcting right")
                robot.drive("rotate_right", speed=CORRECTION_SPEED, ms=CORRECTION_MS)
            elif error < 0:
                print(f"[INFO] line={line_digital} error={error} - correcting left")
                robot.drive("rotate_left", speed=CORRECTION_SPEED, ms=CORRECTION_MS)
            else:
                robot.drive("forward", speed=FORWARD_SPEED, ms=FORWARD_MS)
    except ProgramStopped:
        print("[INFO] precision course stopped - BOOT button disabled the program")
    finally:
        robot.stop()


if __name__ == "__main__":
    # Camera-free, robot-free self-test: a fake robot plays back a scripted
    # sequence of sensor readings so we can assert on the exact drive()
    # sequence, instead of only checking that this module imports.

    class _FakeRobot:
        """sensor_sequence is popped one reading per tick; once exhausted the
        loop ends (is_running() flips False) so tests don't hang forever."""

        def __init__(self, sensor_sequence, fail_at: int = None) -> None:
            self._sensor_sequence = list(sensor_sequence)
            self._running = True
            self.calls = []
            self._call_count = 0
            self._fail_at = fail_at

        def is_running(self) -> bool:
            return self._running

        def read_sensors(self) -> dict:
            if not self._sensor_sequence:
                self._running = False
                return {"line_ok": True, "line_digital": [1, 0, 0, 1]}  # unused - loop exits first
            return self._sensor_sequence.pop(0)

        def drive(self, command, speed=150, ms=500):
            self._call_count += 1
            self.calls.append(("drive", command, speed, ms))
            if self._fail_at is not None and self._call_count == self._fail_at:
                raise ProgramStopped("BOOT button disabled mid-move")
            # The real MiniAutoRobot.drive() blocks for roughly ms/1000s per
            # call (see robot_client.py) - simulate a slice of that so the
            # max_seconds timeout test below actually has real time to bite
            # against, instead of 1000 fake ticks completing instantly.
            time.sleep(0.01)

        def stop(self):
            self.calls.append(("stop",))
            return True

    print("[SELF-TEST] drifting right (error>0) -> rotate_right")
    robot = _FakeRobot([{"line_ok": True, "line_digital": [0, 0, 1, 1]}])  # dot([-3,-1,1,3])=4
    run_precision_course(robot, max_seconds=5)
    assert robot.calls[0] == ("drive", "rotate_right", CORRECTION_SPEED, CORRECTION_MS), robot.calls

    print("[SELF-TEST] drifting left (error<0) -> rotate_left")
    robot = _FakeRobot([{"line_ok": True, "line_digital": [1, 1, 0, 0]}])  # dot=-4
    run_precision_course(robot, max_seconds=5)
    assert robot.calls[0] == ("drive", "rotate_left", CORRECTION_SPEED, CORRECTION_MS), robot.calls

    print("[SELF-TEST] centred (error==0) -> forward")
    robot = _FakeRobot([{"line_ok": True, "line_digital": [1, 0, 0, 1]}])  # dot=-3+3=0
    run_precision_course(robot, max_seconds=5)
    assert robot.calls[0] == ("drive", "forward", FORWARD_SPEED, FORWARD_MS), robot.calls

    print("[SELF-TEST] line lost (all-zero) -> search, not a blind forward crawl")
    robot = _FakeRobot([{"line_ok": True, "line_digital": [0, 0, 0, 0]}])
    run_precision_course(robot, max_seconds=5)
    assert robot.calls[0] == ("drive", SEARCH_DIRECTION, SEARCH_SPEED, SEARCH_MS), robot.calls

    print("[SELF-TEST] bad read (line_ok=False) -> search, even if line_digital looks like an intersection")
    robot = _FakeRobot([{"line_ok": False, "line_digital": [1, 1, 1, 1]}])
    run_precision_course(robot, max_seconds=5)
    assert robot.calls[0] == ("drive", SEARCH_DIRECTION, SEARCH_SPEED, SEARCH_MS), robot.calls

    print("[SELF-TEST] intersection (all-one) -> holds (stop()) before continuing, not just at the end")
    robot = _FakeRobot([{"line_ok": True, "line_digital": [1, 1, 1, 1]}])
    run_precision_course(robot, max_seconds=5)
    assert ("stop",) in robot.calls[:-1], "expected a mid-loop stop() from the intersection hold, not just the finally"
    assert robot.calls[-1] == ("stop",), robot.calls

    print("[SELF-TEST] ProgramStopped mid-move is caught, not raised - and stop() still runs")
    robot = _FakeRobot([{"line_ok": True, "line_digital": [1, 0, 0, 1]}], fail_at=1)
    run_precision_course(robot, max_seconds=5)  # must NOT raise
    assert robot.calls == [("drive", "forward", FORWARD_SPEED, FORWARD_MS), ("stop",)], robot.calls

    print("[SELF-TEST] max_seconds timeout ends the loop even if is_running() stays True")
    robot = _FakeRobot([{"line_ok": True, "line_digital": [1, 0, 0, 1]}] * 1000)  # never runs out on its own
    run_precision_course(robot, max_seconds=0.05)
    assert robot.calls[-1] == ("stop",), robot.calls
    assert len(robot.calls) < 1000, "max_seconds must cut the loop short, not run all 1000 scripted ticks"
    print(f"  -> stopped after {len(robot.calls)} tick(s) within the 0.05s ceiling")

    print("SELF-TEST PASSED")
