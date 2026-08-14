"""
eval_frames.py  -  score an exported .eim model against saved, labelled frames.

WHY THIS EXISTS. The `goal` class is the weak point of both trained models
(see capture.py's note - it is why button C was repointed to goal captures at
the venue). soccer_policy.py's own-goal avoidance is gated entirely behind a
`goal` detection, so goal-class errors do not degrade that system gracefully -
they change which branch runs:

  FALSE NEGATIVE (real goal missed)  - policy loses goal_side, peeks for
      POSSESSION_SCAN_GRACE_TICKS, then pushes forward BLIND at
      CAUTIOUS_PUSH_SPEED regardless of which goal is ahead. Facing our own
      goal, that is an own goal.
  FALSE POSITIVE (goal reported where there is none) - wall_detector gets
      called on a frame with no goal, returns a confident colour, and the
      policy acts on it at full speed. GOAL_MEMORY_TICKS then extends one bad
      frame's influence over the next several ticks.

So "what is this model's goal-class FP/FN rate on real frames from this
field" is a direct input to how much the policy should trust it - not a
generic accuracy question. Studio's mAP does not answer it, because Studio
scores the validation split, not the venue.

USAGE (on the robot's Linux side - .eim files are aarch64 Linux binaries and
will not run on a laptop):

    python3 python/eval_frames.py --model models/soccer-pico-160.eim --frames captures/labelled
    python3 python/eval_frames.py --model models/soccer-nano-192.eim --frames captures/labelled

Run it twice, once per model, and compare. Same frames both times, so it is a
fair A/B rather than two separate live sessions with different framing.

GROUND TRUTH comes from the filename, no sidecar files to keep in sync:
a frame whose name contains "goal" is expected to contain a goal; anything
else (e.g. "wall.png", "empty-corner.png") is expected NOT to. That matches
the naming already in use for the venue captures, e.g.

    blue goal from red.png    -> goal expected
    red goal from blue.png    -> goal expected
    wall.png                  -> no goal expected

Add "ball" to a filename to assert a ball is expected too; frames with
"noball" in the name assert the opposite. Ball expectations are optional -
frames that say nothing about the ball are simply not scored on it.
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

try:
    import cv2
except ImportError:
    cv2 = None


def expectations_from_name(name: str) -> dict:
    """Parse ground truth out of a filename. Returns {"goal": bool,
    "ball": bool|None} - None meaning "this frame makes no claim, don't
    score it"."""
    low = name.lower()
    goal_expected = "goal" in low
    if "noball" in low or "no ball" in low or "no-ball" in low:
        ball_expected = False
    elif "ball" in low:
        ball_expected = True
    else:
        ball_expected = None
    return {"goal": goal_expected, "ball": ball_expected}


def _verdict(expected: bool, detected: bool) -> str:
    if expected and detected:
        return "ok"
    if expected and not detected:
        return "FN"
    if not expected and detected:
        return "FP"
    return "ok"


def evaluate(model_path: str, frames_dir: str, min_confidence: float) -> int:
    if cv2 is None:
        print("[FAIL] opencv-python is not installed - run this on the robot's Linux side")
        return 2

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    try:
        from ei_runner import ObjectDetector, ModelUnavailable
    except ImportError as exc:
        print(f"[FAIL] could not import ei_runner ({exc})")
        return 2

    paths = sorted(
        p for ext in ("png", "jpg", "jpeg")
        for p in glob.glob(os.path.join(frames_dir, f"*.{ext}"))
    )
    if not paths:
        print(f"[FAIL] no images found in {frames_dir}")
        return 2

    print(f"model  : {model_path}")
    print(f"frames : {len(paths)} from {frames_dir}")
    print(f"min_confidence: {min_confidence}\n")

    rows = []
    try:
        with ObjectDetector(model_path, min_confidence=min_confidence) as det:
            for path in paths:
                name = os.path.splitext(os.path.basename(path))[0]
                frame = cv2.imread(path)
                if frame is None:
                    print(f"[WARN] unreadable, skipping: {path}")
                    continue
                exp = expectations_from_name(name)
                try:
                    detections = det.infer(frame)
                except Exception as exc:  # noqa: BLE001 - one bad frame must not end the sweep
                    print(f"[WARN] inference failed on {name}: {exc}")
                    continue
                goal = det.best(detections, "goal")
                ball = det.best(detections, "soccer_ball")
                robot = det.best(detections, "robot")
                rows.append((name, exp, goal, ball, robot))
    except ModelUnavailable as exc:
        print(f"[FAIL] {exc}")
        return 2

    hdr = f"{'frame':<38}{'goal?':<7}{'got':<7}{'':<4}{'ball':<7}{'':<4}{'robot':<7}"
    print(hdr)
    print("-" * len(hdr))
    goal_fn = goal_fp = goal_ok = 0
    ball_fn = ball_fp = 0
    for name, exp, goal, ball, robot in rows:
        gdet = goal is not None
        gv = _verdict(exp["goal"], gdet)
        if gv == "FN":
            goal_fn += 1
        elif gv == "FP":
            goal_fp += 1
        else:
            goal_ok += 1

        bv = ""
        if exp["ball"] is not None:
            bv = _verdict(exp["ball"], ball is not None)
            if bv == "FN":
                ball_fn += 1
            elif bv == "FP":
                ball_fp += 1

        gconf = f"{goal.confidence:.2f}" if goal else "-"
        bconf = f"{ball.confidence:.2f}" if ball else "-"
        rconf = f"{robot.confidence:.2f}" if robot else "-"
        print(
            f"{name[:37]:<38}{str(exp['goal']):<7}{gconf:<7}{gv:<4}{bconf:<7}{bv:<4}{rconf:<7}"
        )

    print("-" * len(hdr))
    total = len(rows)
    print(f"  goal class : {goal_ok}/{total} correct   FN={goal_fn}  FP={goal_fp}")
    if ball_fn or ball_fp:
        print(f"  ball class : FN={ball_fn}  FP={ball_fp}")
    print()
    if goal_fp:
        print(
            f"  {goal_fp} FALSE POSITIVE(s). These are the expensive ones: soccer_policy will call\n"
            f"  wall_detector on a frame with no goal, get a confident colour back, and drive on\n"
            f"  it at full speed - for this tick plus GOAL_MEMORY_TICKS more."
        )
    if goal_fn:
        print(
            f"  {goal_fn} FALSE NEGATIVE(s). Policy will peek for POSSESSION_SCAN_GRACE_TICKS and\n"
            f"  then push forward blind. Harmless facing the opponent's goal; an own goal facing\n"
            f"  our own."
        )
    if not goal_fp and not goal_fn:
        print("  No goal-class errors on this set. Add harder frames before trusting that.")
    return 0 if (goal_fp == 0 and goal_fn == 0) else 1


def _self_test() -> None:
    """Hardware-free check of the filename->truth parsing and the FP/FN
    verdict logic - the two things that would silently mis-score a whole
    sweep. Runs on a plain laptop with no model, no camera, no cv2."""
    cases = {
        "blue goal from red": (True, None),
        "red goal from blue": (True, None),
        "blue goal at the corner of the image": (True, None),
        "wall": (False, None),
        "empty corner": (False, None),
        "goal and ball together": (True, True),
        "wall noball": (False, False),
    }
    for name, (want_goal, want_ball) in cases.items():
        got = expectations_from_name(name)
        assert got["goal"] == want_goal, f"{name!r}: goal expected {want_goal}, parsed {got['goal']}"
        assert got["ball"] == want_ball, f"{name!r}: ball expected {want_ball}, parsed {got['ball']}"
    print(f"[SELF-TEST] filename parsing: {len(cases)} cases OK")

    assert _verdict(True, True) == "ok"
    assert _verdict(True, False) == "FN"
    assert _verdict(False, True) == "FP"
    assert _verdict(False, False) == "ok"
    print("[SELF-TEST] FP/FN verdict logic OK")
    print("SELF-TEST PASSED")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--model", help="path to the .eim model")
    parser.add_argument("--frames", help="directory of labelled frames")
    parser.add_argument("--min-confidence", type=float, default=0.6,
                        help="matches soccer_policy.MIN_CONFIDENCE (default 0.6)")
    parser.add_argument("--self-test", action="store_true",
                        help="run the hardware-free self-test and exit")
    args = parser.parse_args()

    if args.self_test or not (args.model and args.frames):
        if not args.self_test:
            print("[INFO] --model and --frames not both given; running self-test instead.\n")
        _self_test()
        sys.exit(0)

    sys.exit(evaluate(args.model, args.frames, args.min_confidence))
