"""
calibrate_motion.py  -  measure how far this robot actually travels per
millisecond of drive time, so the kickoff opening can be expressed in inches.

soccer_policy's OPENING_SEQUENCE is built from real field geometry (strafe
~6in left to line up on the goal, then ~15in forward to the ball), but
converting inches to motor milliseconds needs a number only this specific
robot, on this specific surface, at this specific battery level can tell us.
This script measures it.

WHEELS DOWN, on the actual field surface - carpet vs. the field's smooth
floor will give different answers, and so will a half-drained battery.

Run:
    python3 python/calibrate_motion.py                 # both axes
    python3 python/calibrate_motion.py --axis strafe   # just the strafe
    python3 python/calibrate_motion.py --ms 800        # longer pulse

It drives ONE bounded pulse per axis and asks you to measure the distance
with a ruler/tape. It then prints the MS_PER_INCH values to paste into
soccer_policy.py. Nothing is written automatically - you paste it, so you
can sanity-check the number first.
"""
from __future__ import annotations

import argparse
import sys

DEFAULT_PULSE_MS = 600


def _measure(robot, label: str, direction: str, speed: int, pulse_ms: int) -> float | None:
    print()
    print("=" * 62)
    print(f"  {label}:  direction={direction}  speed={speed}  duration={pulse_ms}ms")
    print("=" * 62)
    print("  Place the robot with clear space ahead of it in that direction.")
    print("  Mark where it starts (tape, a pen line, the edge of a tile).")
    input("  Press ENTER to drive one pulse (Ctrl+C to skip)... ")

    robot.drive(direction, speed=speed, ms=pulse_ms)

    print("  Now measure how far it moved, in INCHES.")
    raw = input("  Distance in inches (blank to skip this axis): ").strip()
    if not raw:
        print("  skipped")
        return None
    try:
        inches = float(raw)
    except ValueError:
        print(f"  '{raw}' isn't a number - skipping")
        return None
    if inches <= 0:
        print("  distance must be positive - skipping")
        return None

    ms_per_inch = pulse_ms / inches
    print(f"  -> {inches}in in {pulse_ms}ms  =  {ms_per_inch:.1f} ms per inch")
    return ms_per_inch


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axis", choices=("strafe", "forward", "both"), default="both")
    parser.add_argument("--ms", type=int, default=DEFAULT_PULSE_MS, help="pulse duration to test with")
    args = parser.parse_args()

    try:
        from robot_client import MiniAutoRobot
        import soccer_policy as sp
    except ImportError as exc:
        print(f"[FAIL] run this on the robot, inside the app container: {exc}")
        sys.exit(1)

    robot = MiniAutoRobot()
    sensors = robot.read_sensors()
    if not sensors.get("program_enabled"):
        print("[INFO] program is not enabled - press the BOOT button (short press) first,")
        print("       wait for solid green, then re-run this script.")
        sys.exit(1)

    print("WHEELS DOWN. Clear space around the robot. Ctrl+C stops everything.")

    strafe_ms_per_inch = None
    forward_ms_per_inch = None

    try:
        if args.axis in ("strafe", "both"):
            strafe_ms_per_inch = _measure(
                robot, "STRAFE LEFT", "left", sp.OPENING_STRAFE_SPEED, args.ms
            )
        if args.axis in ("forward", "both"):
            forward_ms_per_inch = _measure(
                robot, "FORWARD", "forward", sp.OPENING_FORWARD_SPEED, args.ms
            )
    except KeyboardInterrupt:
        print("\n[INFO] interrupted")
    finally:
        robot.stop()

    print()
    print("=" * 62)
    print("  PASTE INTO python/soccer_policy.py")
    print("=" * 62)
    if strafe_ms_per_inch is not None:
        print(f"STRAFE_MS_PER_INCH = {strafe_ms_per_inch:.1f}")
    else:
        print(f"STRAFE_MS_PER_INCH = {sp.STRAFE_MS_PER_INCH}   # unchanged (not measured)")
    if forward_ms_per_inch is not None:
        print(f"FORWARD_MS_PER_INCH = {forward_ms_per_inch:.1f}")
    else:
        print(f"FORWARD_MS_PER_INCH = {sp.FORWARD_MS_PER_INCH}   # unchanged (not measured)")

    s = strafe_ms_per_inch if strafe_ms_per_inch is not None else sp.STRAFE_MS_PER_INCH
    f = forward_ms_per_inch if forward_ms_per_inch is not None else sp.FORWARD_MS_PER_INCH
    strafe_ms = int(sp.OPENING_LEFT_INCHES * s)
    fwd_in = max(sp.OPENING_FORWARD_INCHES - sp.OPENING_FORWARD_STOP_SHORT_INCHES, 1)
    fwd_ms = int(fwd_in * f)
    print()
    print(f"  With these, the opening becomes:")
    print(f"    strafe left {sp.OPENING_LEFT_INCHES}in  -> {strafe_ms}ms")
    print(f"    forward     {fwd_in}in -> {fwd_ms}ms   "
          f"({sp.OPENING_FORWARD_INCHES}in to the ball, stopping {sp.OPENING_FORWARD_STOP_SHORT_INCHES}in short)")
    if strafe_ms > 5000 or fwd_ms > 5000:
        print()
        print("  [WARN] firmware clamps a single drive() to 5000ms - a value above that")
        print("         will be truncated. Raise the speed rather than the duration.")


if __name__ == "__main__":
    main()
