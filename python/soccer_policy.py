"""
soccer_policy.py  -  match-play decision loop for the Robot Soccer Cup.

Implements the developer guide's recommended AI application loop:

    Acquire -> Preprocess -> Infer -> Validate -> Decide -> Act -> Reobserve

`SoccerPolicy.decide_and_act()` is the "Decide -> Act" half of one tick. It is
handed one already-Acquired/Preprocessed camera frame plus one fresh sensors
read, runs Inference (ei_runner.ObjectDetector), Validates the detections
against MIN_CONFIDENCE / staleness / safety, Decides what to do, and executes
AT MOST ONE bounded MiniAutoRobot action before returning. The caller (see
play_match.py) owns the outer while-loop: it Reobserves (grabs a fresh frame
and a fresh sensors read) and calls decide_and_act() again next tick.

THREE PIECES OF NON-OBVIOUS DESIGN IN THIS FILE, READ BEFORE TUNING:

1. HYBRID BALL TRACKING (search "_BallTracker" below). A raw "ball visible /
   not visible" split means one bad frame (motion blur, the ball briefly
   behind our own chassis, a dropped camera frame) makes the robot give up
   and start spinning to search, even though the ball almost certainly
   didn't teleport. Real robot-soccer teams (RoboCup Junior Soccer and
   friends) handle this with a `tracking -> coasting -> lost` state machine:
   while the ball is visible, track it normally; the instant it disappears,
   keep believing a short-lived extrapolation of where it should still be
   (based on a smoothed velocity estimate) for a few ticks ("coasting")
   before conceding it's actually `lost` and falling back to search. This is
   a lightweight alpha-beta/EMA-smoothed tracker, not a full Kalman filter -
   deliberately: FOMO's 96x96 grid output is coarse enough that a Kalman
   filter's covariance modelling has little real signal to exploit, and a
   simpler tracker is something the team can actually reason about and
   retune tomorrow with real numbers, not a black box. When search finally
   does kick in, it's seeded toward the side the ball was last seen on
   (mirrors the "restart near the last known position instead of a blind
   sweep" approach used in RoboCup Junior Soccer ball-recovery strategies),
   instead of a context-free coin flip.

2. POSSESSION-SAFE SCANNING (search "POSSESSION-SAFE" below). We must never
   push the ball toward our OWN goal, so before committing to a full-speed
   push the policy needs to know which end of the field it faces. That comes
   from wall_detector.WallSideDetector reading the field tape, compared
   against our own team colour (hold_toggle). The policy is DELIBERATELY
   CONSERVATIVE: it only commits to a full-speed push when the end ahead is
   POSITIVELY confirmed as the opponent's. If the tape can't be read at all,
   it does NOT default to pushing. Instead it protects the ball (no
   retreating, no driving away - that would surrender it) with small bounded
   "peek" moves, for a bounded grace period, before cautiously proceeding at
   reduced speed if that period runs out unresolved. HOW LONG TO STAY
   CAUTIOUS BEFORE PROCEEDING ANYWAY (POSSESSION_SCAN_GRACE_TICKS) IS A REAL
   RISK-TOLERANCE CALL THE TEAM SHOULD OWN - see the constant below.

   NOTE: this used to be gated behind a "goal" detection - the wall was only
   read on ticks where the model reported a goal. That gate is gone; see
   design note 7 for why it was actively harmful.

3. TWO-TIER OPPONENT CONTACT (search "opponent contesting" below). The
   Game Rules' Yellow Card conditions are CORNERING the opponent against a
   wall, or TIPPING them over - NOT mere proximity. An earlier version of
   this file treated "opponent is close" as a reason to retreat, which is
   both wrong (ordinary contested-ball proximity isn't a foul) and
   exploitable (the other team could camp near the ball/us and force us to
   keep backing off, effectively taking the ball without ever committing a
   foul themselves). This version only retreats for genuinely imminent
   contact (COLLISION_IMMINENT_MM, tight) and instead JUKES sideways around
   the opponent - keeps contesting the ball - for ordinary contested
   closeness (CONTESTED_MM, wider). All four opponent-contact constants
   below are unvalidated guesses made with zero real ultrasonic/opponent
   data - retune them tomorrow against how your actual robot and opponents
   behave, and lean AWAY from soft/skittish if anything - a robot that
   never contests the ball can't win a 1v1.

7. THE WALL IS READ EVERY TICK, NOT ONLY WHEN A GOAL IS DETECTED (search
   "design note 7" below). Two different questions were previously welded
   together: "is there a goal in view?" (the model's job, and the weakest
   output it has - see capture.py's venue note) and "which end of the field
   am I facing?" (wall_detector's job). The second was gated behind the
   first, which turned every goal-class error into a policy error:

     FALSE NEGATIVE - a real goal missed meant goal_side went unresolved,
       the policy peeked for POSSESSION_SCAN_GRACE_TICKS, then pushed
       forward BLIND at CAUTIOUS_PUSH_SPEED with no idea which goal was
       ahead. Facing our own net, that is an own goal - i.e. a missed
       detection did not degrade own-goal avoidance, it SKIPPED it.
     FALSE POSITIVE - a goal reported where there was none meant the wall
       got classified on a frame with no goal in it, returned a confident
       answer, and the policy drove on it at full speed. Worse, the memory
       window below then extended one bad frame's influence over several
       ticks.

   But the second question never needed the first. The tape runs down the
   SIDE walls, visible almost everywhere on the field - a goal does not have
   to be in frame to know which way we're pointing. So the wall is now read
   unconditionally at the decision point and the goal detection is advisory
   only: logged, never steered on. Both failure modes above stop being able
   to cause a wrong turn.

   Cost is ~0.5ms/tick measured (~3ms budgeted for the UNO Q's slower CPU)
   against a tick dominated by drive()'s 300-350ms block, and it REMOVES the
   6-tick peek in the common case - so it is a net speed win of roughly 75x,
   not a tradeoff. It does not touch ball detection or aiming: those branches
   return before reaching this code.

Only project-local modules (ei_runner, wall_detector, robot_client) are
imported here, and only under TYPE_CHECKING - decide_and_act() only ever
touches the objects it's handed through duck typing (frame.shape,
detection.center_x, detector.infer/best, wall.classify/field_line). That
keeps this module importable and py_compile-able on a plain laptop with no
robot, camera, cv2, numpy, or edge_impulse_linux installed - none of those
are needed until decide_and_act() actually runs on-device tomorrow.
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from ei_runner import Detection, ObjectDetector
    from robot_client import MiniAutoRobot
    from wall_detector import WallSideDetector

# --- Tunables: retune here without touching the decision logic below ------

# Detections below this confidence are treated as "not seen" even though
# ei_runner.ObjectDetector may have its own (possibly looser) internal
# min_confidence - this is the policy's own, independently-tunable bar.
MIN_CONFIDENCE = 0.6

# Never drive on a camera frame older than this - a stale frame means we are
# steering blind, which is worse than not moving.
FRAME_STALE_SEC = 0.75

# Fraction of frame width the ball's centre may drift from the frame centre
# before we bother turning. Too tight -> constant twitchy turning; too loose
# -> the robot drives past the ball instead of pushing it straight.
CENTER_DEADZONE_FRAC = 0.12

APPROACH_SPEED = 150   # matches main.py's default drive() speed
SEARCH_SPEED = 120     # slower scan speed while genuinely searching
MIN_APPROACH_SPEED = 80  # floor so the final-approach slowdown never stalls the motors

TURN_MS = 200   # bounded rotate pulse (aim / search)
DRIVE_MS = 250  # bounded forward/backward pulse (approach / disengage)

# --- Hybrid ball tracking (see design note 1 above) ------------------------

# EMA smoothing weight on each new velocity sample (0..1). Higher = tracks
# genuine speed changes faster but is noisier against FOMO's coarse 96x96
# grid quantization; lower = smoother but slower to react to real direction
# changes. 0.5 is a straight-down-the-middle starting guess.
BALL_VELOCITY_EMA_ALPHA = 0.5

# How many CONSECUTIVE ticks without a real ball detection we keep trusting
# the extrapolated position before conceding the ball is actually "lost" and
# falling back to search. Too high risks confidently steering at empty space
# for too long; too low throws away tracking on every single dropped frame.
# At roughly 200-250ms/tick this is ~0.6-0.75s of "benefit of the doubt".
COAST_TICKS = 3

# Reduced forward speed while "coasting" (centred prediction, not a confirmed
# detection) - real momentum, but deliberately gentler than a fully-confirmed
# approach, since we're driving on a guess.
COAST_CREEP_SPEED = 100

# --- Possession-safe goal scanning (see design note 2 above) ---------------

POSSESSION_SCAN_SPEED = 110
POSSESSION_SCAN_TURN_MS = 120  # a small "peek" turn, deliberately gentler/shorter than TURN_MS

# How many consecutive "ball held, goal side unresolved" ticks to tolerate
# before cautiously proceeding anyway rather than stalling forever. ~6 ticks
# is roughly 1.2-1.5s. THIS IS A RISK-TOLERANCE DIAL, NOT A PHYSICS CONSTANT -
# lower it if the team would rather risk stalling than risk an own goal;
# raise it if stalling in front of an unresolved goal feels worse in
# practice. Decide this with the team tomorrow, don't just trust my guess.
POSSESSION_SCAN_GRACE_TICKS = 6

# Reduced push speed used only once the grace period above has expired
# without resolving which goal is which.
CAUTIOUS_PUSH_SPEED = 90

# --- Opponent contact: two-tier response (see design note 3 above) --------

# Below this ultrasonic reading, treat it as genuinely imminent contact -
# real risk of an accidental ram/tip - and retreat.
COLLISION_IMMINENT_MM = 100

# Below this (wider) ultrasonic reading, treat it as ordinary contested
# closeness - NOT a foul by itself - and juke sideways around the opponent
# instead of surrendering ground.
CONTESTED_MM = 200

# How much of the frame width the "robot" detection must fill before the
# tighter emergency tier fires - deliberately demanding (opponent genuinely
# filling most of the view), so this tier is reserved for real emergencies.
ROBOT_EMERGENCY_WIDTH_FRAC = 0.6

# Looser bar for the juke tier - fine to trigger this more liberally since
# juking is a low-commitment response, not a retreat.
ROBOT_JUKE_WIDTH_FRAC = 0.35

# --- De-wedge safety net (design note 4) ------------------------------------
# True "are we stuck" detection isn't reliable with this sensor suite: the
# ultrasonic reading to a ball we're successfully pushing looks identical to
# a ball wedged motionless against a wall - both just say "something is
# right in front of me" every tick, whether or not the robot is actually
# translating through space. No wheel encoders means no way to tell
# progress from a stall. Rather than chase an unreliable "detect stuck"
# heuristic, this is a cheap, periodic insurance nudge instead: after this
# many CONSECUTIVE confirmed pushes toward a confirmed opponent goal,
# peel off sideways once before resuming. Costs almost nothing against a
# normal fast push (which should reach the goal well before this triggers
# on a field this size); could save an entire match stalled against a wall
# if a push genuinely isn't going anywhere. UNVALIDATED GUESS - retune
# tomorrow once you can see how long a real successful push actually takes.
DEWEDGE_PUSH_TICKS = 10
DEWEDGE_SPEED = 130
DEWEDGE_MS = 200

# --- Escalating search (design note 5) --------------------------------------
# Plain in-place alternating rotation is probably enough to reacquire the
# ball fast on a field this small - but costs nothing to widen the pattern
# with an occasional strafe if a search has genuinely been failing for a
# while, matching the "patrol wider when initial search fails" approach
# documented for RoboCup Junior Soccer ball recovery.
SEARCH_ESCALATE_AFTER_TICKS = 15
SEARCH_WIDEN_EVERY_TICKS = 5

# --- Goal-side memory (design note 6) ---------------------------------------
# Added live at the venue: the moment the robot is closest to actually
# scoring - ball centred, close, right in front of the goal - is exactly the
# moment the ball itself is most likely to be occluding the goal from the
# camera's view. Without this, a goal that drops out of view on the final
# approach gets treated as fully unresolved and the policy starts peeking
# side to side, undoing a confirmation it just made a moment ago. Trust a
# recently-confirmed goal_side for a short window after the goal itself
# stops being detected, instead of immediately discarding it. Invalidated
# the instant the ball itself is lost (see track_state == "lost" below) -
# this is specifically about surviving OUR OWN ball blocking the view, not
# about remembering goal position after looking somewhere else entirely.
GOAL_MEMORY_TICKS = 5


class _BallTracker:
    """Lightweight alpha-beta/EMA-style tracker: smooths a velocity estimate
    from consecutive real ball detections, and for a short window after the
    ball disappears, extrapolates where it should still be instead of
    immediately conceding it's lost. See design note 1 in the module
    docstring for why this isn't a full Kalman filter.
    """

    def __init__(self, coast_ticks: int = COAST_TICKS, velocity_alpha: float = BALL_VELOCITY_EMA_ALPHA) -> None:
        self._coast_ticks = coast_ticks
        self._alpha = velocity_alpha
        self.last_center_x: Optional[float] = None
        self.last_width: Optional[float] = None
        self.velocity_x: float = 0.0  # px/tick, EMA-smoothed
        self.ticks_since_seen: int = 0

    def update(self, ball: Optional["Detection"]) -> str:
        """Feed one tick's ball detection (or None). Returns "tracking" (a
        real detection this tick), "coasting" (recently lost, still within
        the extrapolation window), or "lost" (window exceeded)."""
        if ball is not None:
            if self.last_center_x is not None and self.ticks_since_seen <= self._coast_ticks:
                # Continuous-enough track (including a short coast gap) -
                # trust a velocity estimate from the position delta.
                gap = self.ticks_since_seen + 1
                raw_velocity = (ball.center_x - self.last_center_x) / gap
                self.velocity_x = self._alpha * raw_velocity + (1 - self._alpha) * self.velocity_x
            else:
                # First-ever sighting, or reacquired after being fully lost -
                # a position delta across an untracked gap (we may have been
                # rotating to search) isn't a trustworthy velocity. Don't
                # fabricate one.
                self.velocity_x = 0.0
            self.last_center_x = ball.center_x
            self.last_width = ball.width
            self.ticks_since_seen = 0
            return "tracking"

        self.ticks_since_seen += 1
        if self.last_center_x is None or self.ticks_since_seen > self._coast_ticks:
            return "lost"
        return "coasting"

    def predicted_center_x(self, frame_w: float) -> float:
        assert self.last_center_x is not None, "predicted_center_x() called with no prior sighting"
        predicted = self.last_center_x + self.velocity_x * self.ticks_since_seen
        return max(0.0, min(frame_w, predicted))

    def predicted_width(self) -> float:
        assert self.last_width is not None, "predicted_width() called with no prior sighting"
        return self.last_width

    def last_known_side_left(self, frame_w: float) -> bool:
        """True if the ball was last seen left of frame centre - used to
        seed which way to start searching once fully lost, instead of a
        context-free coin flip. Arbitrarily defaults to True (search left
        first) if the ball has never been seen at all this match."""
        if self.last_center_x is None:
            return True
        return self.last_center_x < frame_w / 2.0


class SoccerPolicy:
    """Holds the small bits of state that must persist across ticks (the
    ball tracker, and the search/scan alternation flags). Everything else is
    passed in fresh each call, per the Acquire/Reobserve loop."""

    def __init__(self, robot: "MiniAutoRobot") -> None:
        self.robot = robot
        self.reset()

    def reset(self) -> None:
        """Clear every piece of cross-tick state. Call this at the start of
        every fresh program-enabled session (see play_match.py), not just at
        construction - after a Yellow Card reset, a ref-initiated pause, or
        simply pressing BOOT again between practice runs, the robot and ball
        have very likely been physically repositioned. A remembered ball
        velocity/position, search-direction bias, or possession-scan/push
        counter from before that reset describes a world state that no
        longer exists, and acting on it (e.g. confidently "coasting" toward
        where the ball used to be) is worse than just starting fresh."""
        self._tracker = _BallTracker()
        self._search_turn_left_next: Optional[bool] = None  # seeded from last-known side on each fresh loss
        self._scan_turn_left_next = True  # alternates the possession-safe "peek" turn
        self._possession_scan_ticks = 0
        self._consecutive_push_ticks = 0  # see DEWEDGE_PUSH_TICKS
        self._last_goal_side: Optional[str] = None  # see GOAL_MEMORY_TICKS
        self._last_goal_side_age = 0

    def decide_and_act(
        self,
        frame_bgr,
        sensors: dict,
        detector: "ObjectDetector",
        wall: "WallSideDetector",
        hold_toggle: bool,
        frame_ts: Optional[float] = None,
    ) -> None:
        """Run one Decide -> Act tick. Executes at most one bounded
        MiniAutoRobot action, then returns so the caller can Reobserve."""
        now = time.monotonic()

        # 1) Safe default: never act while the program is disabled.
        if not sensors.get("program_enabled"):
            return

        # 2) Never drive on missing or stale vision.
        if frame_bgr is None:
            print("[WARN] soccer_policy: no camera frame available - stopping")
            self.robot.stop()
            return
        if frame_ts is None:
            frame_ts = now
        elif (now - frame_ts) > FRAME_STALE_SEC:
            print(f"[WARN] soccer_policy: frame is {now - frame_ts:.2f}s stale - stopping")
            self.robot.stop()
            return

        frame_h, frame_w = frame_bgr.shape[0], frame_bgr.shape[1]
        if frame_w <= 0 or frame_h <= 0:
            print("[WARN] soccer_policy: empty frame - stopping")
            self.robot.stop()
            return

        # 3) Infer, then Validate against our own confidence bar.
        try:
            detections = detector.infer(frame_bgr)
        except Exception as exc:  # noqa: BLE001 - one bad frame must not crash the match loop
            print(f"[WARN] soccer_policy: inference failed ({exc}) - stopping")
            self.robot.stop()
            return

        opponent = self._above_threshold(detector.best(detections, "robot"))
        raw_ball = self._above_threshold(detector.best(detections, "soccer_ball"))
        track_state = self._tracker.update(raw_ball)

        # 4) --- OPPONENT CONTACT: two tiers, checked before ball-chase ----
        # See design note 3: only genuinely imminent contact retreats;
        # ordinary contested closeness jukes around instead.
        if opponent is not None:
            ultrasonic_mm = sensors.get("ultrasonic_mm", -1)
            ultrasonic_valid = ultrasonic_mm > 0
            bbox_w_frac = opponent.width / frame_w

            if ultrasonic_valid and ultrasonic_mm < COLLISION_IMMINENT_MM and bbox_w_frac >= ROBOT_EMERGENCY_WIDTH_FRAC:
                print(
                    f"[WARN] opponent imminent contact (ultrasonic_mm={ultrasonic_mm} "
                    f"bbox_w_frac={bbox_w_frac:.2f}) - emergency back-off"
                )
                self._possession_scan_ticks = 0
                self._consecutive_push_ticks = 0
                self.robot.drive("backward", speed=APPROACH_SPEED, ms=DRIVE_MS)
                return

            if ultrasonic_valid and ultrasonic_mm < CONTESTED_MM and bbox_w_frac >= ROBOT_JUKE_WIDTH_FRAC:
                juke_direction = "right" if opponent.center_x < frame_w / 2.0 else "left"
                print(
                    f"[POLICY] opponent contesting (ultrasonic_mm={ultrasonic_mm} "
                    f"bbox_w_frac={bbox_w_frac:.2f}) - juking {juke_direction} instead of retreating"
                )
                self._possession_scan_ticks = 0
                self._consecutive_push_ticks = 0
                self.robot.drive(juke_direction, speed=APPROACH_SPEED, ms=TURN_MS)
                return

        # 5) --- BALL CHASE: tracking / coasting / lost ---------------------
        if track_state == "lost":
            self._possession_scan_ticks = 0
            self._consecutive_push_ticks = 0
            self._last_goal_side = None  # ball itself is gone, not just occluded - don't trust old goal memory
            if self._search_turn_left_next is None:
                self._search_turn_left_next = self._tracker.last_known_side_left(frame_w)
            direction = "rotate_left" if self._search_turn_left_next else "rotate_right"
            self._search_turn_left_next = not self._search_turn_left_next

            # Escalating search (design note 5): once genuinely lost for a
            # while (past the coast window, then past this extra grace
            # period too), mix in an occasional strafe instead of only
            # rotating in place forever, to cover more of the frame.
            ticks_lost = self._tracker.ticks_since_seen
            if ticks_lost > SEARCH_ESCALATE_AFTER_TICKS and ticks_lost % SEARCH_WIDEN_EVERY_TICKS == 0:
                strafe_direction = "left" if direction == "rotate_left" else "right"
                print(
                    f"[POLICY] ball lost for {ticks_lost} ticks - widening search with a strafe {strafe_direction}"
                )
                self.robot.drive(strafe_direction, speed=SEARCH_SPEED, ms=TURN_MS)
                return

            print(f"[POLICY] ball lost - searching {direction} (biased from last known side)")
            self.robot.drive(direction, speed=SEARCH_SPEED, ms=TURN_MS)
            return

        # We have SOME estimate of the ball (real or short-coast prediction) -
        # clear the search seed so the next loss re-biases fresh.
        self._search_turn_left_next = None

        if track_state == "tracking":
            ball_x, ball_w = raw_ball.center_x, raw_ball.width
        else:  # "coasting"
            ball_x, ball_w = self._tracker.predicted_center_x(frame_w), self._tracker.predicted_width()

        offset = ball_x - frame_w / 2.0
        if abs(offset) > CENTER_DEADZONE_FRAC * frame_w:
            direction = "rotate_left" if offset < 0 else "rotate_right"
            self._possession_scan_ticks = 0
            self._consecutive_push_ticks = 0
            label = "predicted " if track_state == "coasting" else ""
            print(f"[POLICY] {label}ball off-centre (offset={offset:.0f}px) - turning {direction}")
            self.robot.drive(direction, speed=APPROACH_SPEED, ms=TURN_MS)
            return

        if track_state == "coasting":
            # Centred but not confirmed THIS tick - creep cautiously on the
            # prediction rather than committing to a full-speed push.
            self._possession_scan_ticks = 0
            self._consecutive_push_ticks = 0
            print(
                f"[POLICY] predicted ball centred (coasting {self._tracker.ticks_since_seen} "
                "tick(s)) - cautious creep"
            )
            self.robot.drive("forward", speed=COAST_CREEP_SPEED, ms=DRIVE_MS)
            return

        # track_state == "tracking" and centred: the scoring decision point.
        # --- OWN-GOAL-AVOIDANCE / POSSESSION-SAFE SCANNING (design note 2) -
        # Read the wall on EVERY tick that reaches this decision point - NOT
        # only when the model happened to report a goal. See design note 7.
        classification = wall.classify(frame_bgr, team_is_blue=hold_toggle)
        wall_side = classification["result"]  # "OWN SIDE" / "OPPONENT SIDE" / "UNKNOWN"
        print(wall.field_line(classification))

        # The goal detection is now ADVISORY ONLY - logged, never steered on.
        # The `goal` class is the weakest output of both trained models, and
        # gating the own-goal decision behind it meant one missed detection
        # skipped own-goal avoidance entirely (design note 7).
        goal = self._above_threshold(detector.best(detections, "goal"))
        if goal is not None:
            print(f"[POLICY] goal in view (conf={goal.confidence:.2f}) - advisory, not steered on")

        if wall_side in ("OWN SIDE", "OPPONENT SIDE"):
            goal_side = wall_side
            # Only DECISIVE readings are remembered. Storing "UNKNOWN" here
            # would let a single glare/bad-angle frame overwrite a good
            # confirmation, which is strictly worse than keeping the old one.
            self._last_goal_side = wall_side
            self._last_goal_side_age = 0
        elif self._last_goal_side is not None and self._last_goal_side_age < GOAL_MEMORY_TICKS:
            # Wall unreadable THIS tick (glare, bad angle, tape out of the
            # detector's band) but we had a decisive reading moments ago and
            # the robot cannot have spun around in that time. Trust it briefly
            # rather than discarding a good answer over one bad frame.
            self._last_goal_side_age += 1
            goal_side = self._last_goal_side
            print(
                f"[POLICY] wall unreadable this tick (age={self._last_goal_side_age}/{GOAL_MEMORY_TICKS}) "
                f"- trusting recent reading: {goal_side}"
            )
        else:
            goal_side = None
            self._last_goal_side = None

        if goal_side == "OWN SIDE":
            print("[POLICY] ball centred but OWN goal is ahead - peeling off, not pushing")
            self._possession_scan_ticks = 0
            self._consecutive_push_ticks = 0
            self.robot.drive("right", speed=SEARCH_SPEED, ms=TURN_MS)
            return

        if goal_side == "OPPONENT SIDE":
            self._possession_scan_ticks = 0

            # De-wedge safety net (design note 4): after enough CONSECUTIVE
            # confirmed pushes with nothing else interrupting, insert one
            # cheap sideways nudge before resuming, on the theory that a
            # push that were going to succeed usually would have by now.
            self._consecutive_push_ticks += 1
            if self._consecutive_push_ticks > DEWEDGE_PUSH_TICKS:
                self._consecutive_push_ticks = 0
                nudge_direction = "left" if self._scan_turn_left_next else "right"
                self._scan_turn_left_next = not self._scan_turn_left_next
                print(
                    f"[POLICY] {DEWEDGE_PUSH_TICKS}+ consecutive pushes with no resolution - "
                    f"de-wedge nudge {nudge_direction} in case the ball is pinned against something"
                )
                self.robot.drive(nudge_direction, speed=DEWEDGE_SPEED, ms=DEWEDGE_MS)
                return

            size_frac = min(ball_w / frame_w, 1.0)
            speed = max(int(APPROACH_SPEED * (1.0 - size_frac)), MIN_APPROACH_SPEED)
            print(f"[POLICY] ball centred (size_frac={size_frac:.2f}) - approaching opponent goal at speed={speed}")
            self.robot.drive("forward", speed=speed, ms=DRIVE_MS)
            return

        # goal_side is None (no goal visible) or "UNKNOWN" (goal visible but
        # tape not classifiable) - NOT confirmed safe to push. Protect the
        # ball (no retreat, no drive-away - that surrenders it) while
        # peeking to try to resolve which goal this is, for a bounded grace
        # period before cautiously proceeding anyway.
        self._consecutive_push_ticks = 0
        self._possession_scan_ticks += 1
        if self._possession_scan_ticks > POSSESSION_SCAN_GRACE_TICKS:
            print(
                f"[POLICY] possession-safe scan timed out after {self._possession_scan_ticks} "
                f"tick(s) with goal_side={goal_side!r} - cautiously proceeding"
            )
            self.robot.drive("forward", speed=CAUTIOUS_PUSH_SPEED, ms=DRIVE_MS)
            return

        direction = "left" if self._scan_turn_left_next else "right"
        self._scan_turn_left_next = not self._scan_turn_left_next
        print(
            f"[POLICY] ball held, goal_side={goal_side!r} unresolved "
            f"(scan {self._possession_scan_ticks}/{POSSESSION_SCAN_GRACE_TICKS}) - peeking {direction}"
        )
        self.robot.drive(direction, speed=POSSESSION_SCAN_SPEED, ms=POSSESSION_SCAN_TURN_MS)

    @staticmethod
    def _above_threshold(detection: Optional["Detection"]) -> Optional["Detection"]:
        if detection is None or detection.confidence < MIN_CONFIDENCE:
            return None
        return detection


if __name__ == "__main__":
    # Camera-free, robot-free, model-free self-test: exercises every branch
    # (including the hybrid tracker's tracking/coasting/lost states and the
    # possession-safe scan) using lightweight stand-ins, so it runs tonight
    # on a plain laptop with nothing attached.

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
        """Returns the SAME fixed detections every call - fine for
        single-tick scenarios; use _ScriptedDetector below for multi-tick
        tracker tests where what's "seen" needs to change tick to tick."""

        def __init__(self, detections) -> None:
            self.detections = detections

        def infer(self, frame_bgr):
            return self.detections

        def best(self, detections, label):
            matches = [d for d in detections if d.label == label]
            return max(matches, key=lambda d: d.confidence) if matches else None

    class _ScriptedDetector:
        """Pops the next scripted detection list on every infer() call."""

        def __init__(self, script) -> None:
            self._script = list(script)

        def infer(self, frame_bgr):
            assert self._script, "story ran out of scripted ticks before the policy did"
            return self._script.pop(0)

        def best(self, detections, label):
            matches = [d for d in detections if d.label == label]
            return max(matches, key=lambda d: d.confidence) if matches else None

    class _FakeWallDetector:
        def __init__(self, result: str) -> None:
            self.result = result

        def classify(self, frame_bgr, team_is_blue: bool) -> dict:
            wall = "RED" if self.result == "OWN SIDE" else "BLUE" if self.result == "OPPONENT SIDE" else "UNKNOWN"
            return {"side": wall, "red_pct": 0.0, "blue_pct": 0.0, "team": "RED", "wall": wall, "result": self.result}

        def field_line(self, classification: dict) -> str:
            return f"[FIELD] (self-test) -> {classification['result']}"

    class _ScriptedWallDetector(_FakeWallDetector):
        """Returns a different verdict per tick - needed now that the wall is
        read EVERY tick (design note 7), so tests that used to simulate "the
        model lost the goal" must instead simulate "the tape became
        unreadable"."""

        def __init__(self, script) -> None:
            super().__init__("UNKNOWN")
            self._script = list(script)

        def classify(self, frame_bgr, team_is_blue: bool) -> dict:
            self.result = self._script.pop(0) if self._script else "UNKNOWN"
            return super().classify(frame_bgr, team_is_blue)

    class _FakeRobot:
        def __init__(self) -> None:
            self.calls = []

        def drive(self, command, speed=150, ms=500):
            self.calls.append(("drive", command, speed, ms))

        def stop(self):
            self.calls.append(("stop",))
            return True

    FRAME = _FakeFrame(240, 320)  # centre x = 160
    ENABLED_SENSORS = {"program_enabled": True, "ultrasonic_mm": -1}

    print("[SELF-TEST] own-goal avoidance: centred ball + OWN SIDE goal -> peel off, never forward")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    ball = _FakeDetection("soccer_ball", 0.9, x=150, y=150, width=20, height=20)  # centre_x=160, centred
    goal = _FakeDetection("goal", 0.9, x=100, y=20, width=120, height=60)
    detector = _FakeDetector([ball, goal])
    wall = _FakeWallDetector("OWN SIDE")
    policy.decide_and_act(FRAME, ENABLED_SENSORS, detector, wall, hold_toggle=False)
    last = robot.calls[-1]
    assert last == ("drive", "right", SEARCH_SPEED, TURN_MS), f"expected the peel-off pulse, got {last}"
    print(f"  -> {last}  (correctly avoided pushing toward our own goal)")

    print("[SELF-TEST] opponent goal confirmed: centred ball + OPPONENT SIDE goal -> forward push allowed")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    wall = _FakeWallDetector("OPPONENT SIDE")
    policy.decide_and_act(FRAME, ENABLED_SENSORS, detector, wall, hold_toggle=False)
    last = robot.calls[-1]
    expected_speed = max(int(APPROACH_SPEED * (1.0 - ball.width / FRAME.shape[1])), MIN_APPROACH_SPEED)
    assert last == ("drive", "forward", expected_speed, DRIVE_MS), f"expected a forward push, got {last}"
    print(f"  -> {last}  (correctly approached the opponent's goal)")

    print("[SELF-TEST] NEW: centred ball, NO goal in view at all -> possession-safe peek, NOT a blind push")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    detector_no_goal = _FakeDetector([ball])  # no goal detection this tick
    policy.decide_and_act(FRAME, ENABLED_SENSORS, detector_no_goal, _FakeWallDetector("UNKNOWN"), hold_toggle=False)
    last = robot.calls[-1]
    assert last[0] == "drive" and last[1] in ("left", "right"), f"expected a possession-safe peek, got {last}"
    assert last[2:] == (POSSESSION_SCAN_SPEED, POSSESSION_SCAN_TURN_MS), last
    print(f"  -> {last}  (protected the ball and peeked instead of pushing blind)")

    print("[SELF-TEST] NEW: centred ball, goal visible but wall UNKNOWN -> possession-safe peek")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    policy.decide_and_act(FRAME, ENABLED_SENSORS, detector, _FakeWallDetector("UNKNOWN"), hold_toggle=False)
    last = robot.calls[-1]
    assert last[0] == "drive" and last[1] in ("left", "right"), last
    print(f"  -> {last}  (tape not visible - scanned instead of guessing)")

    print("[SELF-TEST] NEW: possession-safe scan grace period expires -> cautious reduced-speed push, not a stall")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    for _ in range(POSSESSION_SCAN_GRACE_TICKS):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, detector_no_goal, _FakeWallDetector("UNKNOWN"), hold_toggle=False)
        assert robot.calls[-1][2:] == (POSSESSION_SCAN_SPEED, POSSESSION_SCAN_TURN_MS), (
            "expected to still be peeking during the grace period", robot.calls[-1]
        )
    policy.decide_and_act(FRAME, ENABLED_SENSORS, detector_no_goal, _FakeWallDetector("UNKNOWN"), hold_toggle=False)
    last = robot.calls[-1]
    assert last == ("drive", "forward", CAUTIOUS_PUSH_SPEED, DRIVE_MS), f"expected a cautious push after timeout, got {last}"
    print(f"  -> {last}  (gave up waiting after {POSSESSION_SCAN_GRACE_TICKS} ticks, proceeded cautiously instead of stalling forever)")

    print("[SELF-TEST] ball off-centre (left) -> rotate_left")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    left_ball = _FakeDetection("soccer_ball", 0.9, x=0, y=150, width=20, height=20)  # centre_x=10
    detector = _FakeDetector([left_ball])
    policy.decide_and_act(FRAME, ENABLED_SENSORS, detector, _FakeWallDetector("UNKNOWN"), hold_toggle=True)
    assert robot.calls[-1] == ("drive", "rotate_left", APPROACH_SPEED, TURN_MS), robot.calls

    print("[SELF-TEST] ball off-centre (right) -> rotate_right")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    right_ball = _FakeDetection("soccer_ball", 0.9, x=290, y=150, width=20, height=20)  # centre_x=300
    detector = _FakeDetector([right_ball])
    policy.decide_and_act(FRAME, ENABLED_SENSORS, detector, _FakeWallDetector("UNKNOWN"), hold_toggle=True)
    assert robot.calls[-1] == ("drive", "rotate_right", APPROACH_SPEED, TURN_MS), robot.calls

    print("[SELF-TEST] NEW: hybrid tracking - ball briefly disappears (within COAST_TICKS) -> coasts, does not search")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    b1 = _FakeDetection("soccer_ball", 0.9, x=140, y=150, width=20, height=20)  # centre_x=150
    b2 = _FakeDetection("soccer_ball", 0.9, x=150, y=150, width=20, height=20)  # centre_x=160, drifting right
    script = _ScriptedDetector([[b1], [b2], [], []])  # 2 real sightings (establish rightward velocity), then 2 misses
    wall_unknown = _FakeWallDetector("UNKNOWN")
    policy.decide_and_act(FRAME, ENABLED_SENSORS, script, wall_unknown, hold_toggle=False)  # tick 1: tracking
    policy.decide_and_act(FRAME, ENABLED_SENSORS, script, wall_unknown, hold_toggle=False)  # tick 2: tracking, centred
    action2 = robot.calls[-1]
    policy.decide_and_act(FRAME, ENABLED_SENSORS, script, wall_unknown, hold_toggle=False)  # tick 3: coasting
    action3 = robot.calls[-1]
    assert action3[0] == "drive" and action3[1] in ("forward", "rotate_left", "rotate_right"), (
        "expected a steering/creep action while coasting, not a search", action3
    )
    assert action3[1] != "rotate_left" or action3[2:] != (SEARCH_SPEED, TURN_MS), (
        "must not fall back to the LOST search branch while still within the coast window", action3
    )
    print(f"  -> tick2={action2}  tick3(coasting)={action3}  (kept tracking through a brief occlusion, did not panic-search)")

    print("[SELF-TEST] NEW: hybrid tracking - occlusion outlasts COAST_TICKS -> falls back to search, biased toward last-known side")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    left_seen = _FakeDetection("soccer_ball", 0.9, x=0, y=150, width=20, height=20)  # last seen well left of centre
    misses = [[]] * (COAST_TICKS + 2)
    script = _ScriptedDetector([[left_seen], *misses])
    policy.decide_and_act(FRAME, ENABLED_SENSORS, script, wall_unknown, hold_toggle=False)  # establishes last-known-left
    for _ in range(COAST_TICKS):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, script, wall_unknown, hold_toggle=False)  # still coasting
    policy.decide_and_act(FRAME, ENABLED_SENSORS, script, wall_unknown, hold_toggle=False)  # now fully lost
    last = robot.calls[-1]
    assert last == ("drive", "rotate_left", SEARCH_SPEED, TURN_MS), (
        "ball was last seen on the LEFT - the first search guess should be rotate_left, not a coin flip", last
    )
    print(f"  -> {last}  (search correctly biased toward the side the ball was last seen on)")

    print("[SELF-TEST] opponent EMERGENCY (very close, fills the frame) -> back off")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    close_opponent = _FakeDetection("robot", 0.9, x=20, y=20, width=250, height=200)  # width_frac=0.78
    detector = _FakeDetector([close_opponent, ball])
    emergency_sensors = {"program_enabled": True, "ultrasonic_mm": 80}  # < COLLISION_IMMINENT_MM
    policy.decide_and_act(FRAME, emergency_sensors, detector, wall_unknown, hold_toggle=False)
    assert robot.calls[-1] == ("drive", "backward", APPROACH_SPEED, DRIVE_MS), robot.calls
    print(f"  -> {robot.calls[-1]}  (genuinely imminent contact - retreated)")

    print("[SELF-TEST] NEW: opponent CONTESTED (moderately close) -> jukes sideways, does NOT retreat")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    contested_opponent = _FakeDetection("robot", 0.9, x=20, y=20, width=130, height=100)  # width_frac~0.41, left-of-centre
    detector = _FakeDetector([contested_opponent, ball])
    contested_sensors = {"program_enabled": True, "ultrasonic_mm": 150}  # inside CONTESTED_MM, outside COLLISION_IMMINENT_MM
    policy.decide_and_act(FRAME, contested_sensors, detector, wall_unknown, hold_toggle=False)
    last = robot.calls[-1]
    assert last[1] != "backward", f"ordinary contested proximity must NOT trigger a full retreat, got {last}"
    assert last == ("drive", "right", APPROACH_SPEED, TURN_MS), (
        "opponent was left-of-centre - expected a juke to the right (away from them), got", last
    )
    print(f"  -> {last}  (kept contesting the ball instead of surrendering it - this is the exploit fix)")

    print("[SELF-TEST] opponent visible but far/small -> ignored, ball-chase proceeds normally")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    distant_opponent = _FakeDetection("robot", 0.9, x=10, y=10, width=15, height=15)  # tiny, far away
    detector = _FakeDetector([distant_opponent, ball])
    far_sensors = {"program_enabled": True, "ultrasonic_mm": -1}
    policy.decide_and_act(FRAME, far_sensors, detector, _FakeWallDetector("OPPONENT SIDE"), hold_toggle=False)
    last = robot.calls[-1]
    assert last[1] not in ("backward",), f"a distant opponent must not trigger any contact response, got {last}"
    print(f"  -> {last}  (a merely-visible opponent doesn't spook the policy)")

    print("[SELF-TEST] no ball detected at all (never seen) -> search, arbitrary default side")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    detector = _FakeDetector([])
    policy.decide_and_act(FRAME, ENABLED_SENSORS, detector, wall_unknown, hold_toggle=False)
    policy.decide_and_act(FRAME, ENABLED_SENSORS, detector, wall_unknown, hold_toggle=False)
    first_dir, second_dir = robot.calls[0][1], robot.calls[1][1]
    assert first_dir != second_dir, robot.calls
    print(f"  -> {first_dir} then {second_dir}  (alternates so it doesn't spin one way forever)")

    print("[SELF-TEST] program disabled -> no action at all")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    disabled_sensors = {"program_enabled": False, "ultrasonic_mm": -1}
    policy.decide_and_act(FRAME, disabled_sensors, _FakeDetector([]), wall_unknown, hold_toggle=False)
    assert robot.calls == [], robot.calls

    print("[SELF-TEST] missing frame -> stop(), never drive blind")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    policy.decide_and_act(None, ENABLED_SENSORS, _FakeDetector([]), wall_unknown, hold_toggle=False)
    assert robot.calls == [("stop",)], robot.calls

    print("[SELF-TEST] NEW: de-wedge - sustained confirmed pushes trigger one nudge, then resume pushing")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    opp_goal_detector = _FakeDetector([ball, goal])
    for i in range(DEWEDGE_PUSH_TICKS):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, opp_goal_detector, _FakeWallDetector("OPPONENT SIDE"), hold_toggle=False)
        assert robot.calls[-1][1] == "forward", (
            f"expected a normal push on tick {i + 1}/{DEWEDGE_PUSH_TICKS}, got {robot.calls[-1]}"
        )
    nudge_tick = robot.calls[-1]
    policy.decide_and_act(FRAME, ENABLED_SENSORS, opp_goal_detector, _FakeWallDetector("OPPONENT SIDE"), hold_toggle=False)
    nudge = robot.calls[-1]
    assert nudge[0] == "drive" and nudge[1] in ("left", "right") and nudge[2:] == (DEWEDGE_SPEED, DEWEDGE_MS), (
        f"expected a de-wedge nudge after {DEWEDGE_PUSH_TICKS} consecutive pushes, got {nudge}"
    )
    policy.decide_and_act(FRAME, ENABLED_SENSORS, opp_goal_detector, _FakeWallDetector("OPPONENT SIDE"), hold_toggle=False)
    assert robot.calls[-1][1] == "forward", f"expected pushing to resume right after the nudge, got {robot.calls[-1]}"
    print(f"  -> pushed normally for {DEWEDGE_PUSH_TICKS} ticks, nudged once ({nudge}), then resumed pushing")

    print("[SELF-TEST] NEW: escalating search - widens with a strafe after prolonged loss, not forever in-place")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    never_seen_detector = _FakeDetector([])
    widened = False
    for i in range(SEARCH_ESCALATE_AFTER_TICKS + SEARCH_WIDEN_EVERY_TICKS + 1):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, never_seen_detector, wall_unknown, hold_toggle=False)
        if robot.calls[-1][1] in ("left", "right"):
            widened = True
            break
    assert widened, "expected the search to eventually widen into a strafe, got only rotations: " + str(robot.calls)
    print(f"  -> search widened into a strafe ({robot.calls[-1]}) after prolonged loss, not endless in-place rotation")

    close_ball = _FakeDetection("soccer_ball", 0.9, x=140, y=150, width=60, height=60)  # centre_x=170, close
    opp_goal = _FakeDetection("goal", 0.9, x=100, y=20, width=120, height=60)
    wall_opponent = _FakeWallDetector("OPPONENT SIDE")

    print("[SELF-TEST] design note 7: a MISSED goal no longer loses goal_side - the wall still answers")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    # The false-negative case that used to skip own-goal avoidance entirely: model never reports
    # the goal, but the tape is perfectly readable. Must push immediately at full confirmed speed,
    # NOT peek for POSSESSION_SCAN_GRACE_TICKS and then push blind.
    for _ in range(4):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([close_ball]), wall_opponent, hold_toggle=False)
    assert all(c[1] == "forward" for c in robot.calls), (
        f"a missed goal must not cost us the push - got {[c[1] for c in robot.calls]}"
    )
    assert robot.calls[-1][2] > CAUTIOUS_PUSH_SPEED, (
        f"expected a CONFIRMED-speed push, not the cautious fallback, got {robot.calls[-1]}"
    )
    print(f"  -> {[c[1] for c in robot.calls]} at speed {robot.calls[-1][2]}  (no peek, no blind push)")

    print("[SELF-TEST] design note 7: a MISSED goal facing our OWN net still peels off (the own-goal case)")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    for _ in range(3):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([close_ball]),
                              _FakeWallDetector("OWN SIDE"), hold_toggle=False)
    assert all(c[1] != "forward" for c in robot.calls), (
        f"THIS IS THE OWN-GOAL BUG: pushed forward toward our own net - got {robot.calls}"
    )
    print(f"  -> {[c[1] for c in robot.calls]}  (peeled off every tick, never pushed into our own goal)")

    print("[SELF-TEST] design note 7: a HALLUCINATED goal cannot steer - only the wall does")
    robot_fp = _FakeRobot()
    policy_fp = SoccerPolicy(robot_fp)
    robot_clean = _FakeRobot()
    policy_clean = SoccerPolicy(robot_clean)
    for _ in range(4):
        # identical wall verdict, one run fed a phantom goal detection the other isn't
        policy_fp.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([close_ball, opp_goal]),
                                 _FakeWallDetector("OWN SIDE"), hold_toggle=False)
        policy_clean.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([close_ball]),
                                    _FakeWallDetector("OWN SIDE"), hold_toggle=False)
    assert robot_fp.calls == robot_clean.calls, (
        "a false-positive goal changed behaviour - the goal class must be advisory only\n"
        f"  with phantom goal: {robot_fp.calls}\n  without:           {robot_clean.calls}"
    )
    print(f"  -> identical actions with and without the phantom goal ({[c[1] for c in robot_fp.calls]})")

    print("[SELF-TEST] wall-reading memory - a briefly unreadable tape doesn't undo a good reading")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    # Tick 1 the tape reads cleanly; after that it goes unreadable (glare / bad angle) every tick.
    scripted = _ScriptedWallDetector(["OPPONENT SIDE"] + ["UNKNOWN"] * (GOAL_MEMORY_TICKS + 4))
    policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([close_ball]), scripted, hold_toggle=False)
    assert robot.calls[-1][1] == "forward", f"expected the confirmed push, got {robot.calls[-1]}"
    policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([close_ball]), scripted, hold_toggle=False)
    last = robot.calls[-1]
    assert last[1] == "forward", f"expected memory to bridge one unreadable tick, got {last}"
    print(f"  -> {last}  (kept pushing through one unreadable-tape tick)")
    for _ in range(GOAL_MEMORY_TICKS):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([close_ball]), scripted, hold_toggle=False)
    last = robot.calls[-1]
    assert last[1] in ("left", "right") and last[2:] == (POSSESSION_SCAN_SPEED, POSSESSION_SCAN_TURN_MS), (
        f"expected memory to expire and fall back to possession-safe scanning, got {last}"
    )
    print(f"  -> {last}  (memory correctly expired after {GOAL_MEMORY_TICKS} unreadable ticks)")

    print("[SELF-TEST] wall-reading memory stores only DECISIVE readings - one bad frame can't erase a good one")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    # OPPONENT SIDE, then a single unreadable frame, then unreadable again. If UNKNOWN were stored
    # as if it were a confirmation, the good reading would be gone and this would start peeking.
    scripted = _ScriptedWallDetector(["OPPONENT SIDE", "UNKNOWN", "UNKNOWN"])
    for _ in range(3):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([close_ball]), scripted, hold_toggle=False)
    assert policy._last_goal_side == "OPPONENT SIDE", (
        f"an UNKNOWN reading must not overwrite a good one, got {policy._last_goal_side!r}"
    )
    assert all(c[1] == "forward" for c in robot.calls), f"expected to keep pushing, got {robot.calls}"
    print(f"  -> memory still {policy._last_goal_side!r} after 2 unreadable frames; kept pushing")

    print("[SELF-TEST] NEW: goal-side memory is invalidated once the ball itself is genuinely lost (not just coasting)")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([close_ball, opp_goal]), wall_opponent, hold_toggle=False)
    assert policy._last_goal_side == "OPPONENT SIDE", "expected the confirmation to be remembered"
    # One missing-ball tick alone is "coasting", not "lost" (COAST_TICKS is exactly for this) - the
    # goal memory correctly isn't touched during a brief coast, same as it survives brief goal-only
    # occlusion. Exhaust the whole coast window before the ball is genuinely "lost".
    for _ in range(COAST_TICKS + 1):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([]), wall_opponent, hold_toggle=False)
    assert policy._last_goal_side is None, "losing the ball entirely (past the coast window) must invalidate goal memory"
    print(f"  -> memory survived the {COAST_TICKS}-tick coast window intact, then cleared once genuinely lost")

    print("[SELF-TEST] NEW: reset() clears all cross-tick state - stale tracking doesn't survive a fresh session")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    # Build up real state: track the ball, then lose it (search bias set), then rack up scan/push ticks.
    policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([ball]), wall_unknown, hold_toggle=False)
    assert policy._tracker.last_center_x is not None, "expected the tracker to have real state before reset"
    policy.reset()
    assert policy._tracker.last_center_x is None, "reset() must clear the ball tracker"
    assert policy._search_turn_left_next is None, "reset() must clear the search-direction seed"
    assert policy._possession_scan_ticks == 0, "reset() must clear the possession-scan counter"
    assert policy._consecutive_push_ticks == 0, "reset() must clear the de-wedge push counter"
    assert policy._last_goal_side is None, "reset() must clear the goal-side memory"
    # And behaviourally: immediately after reset(), a never-before-seen ball is "tracking", not "coasting"
    # on stale velocity - i.e. reset() really did forget the old ball, not just its counters.
    robot2 = _FakeRobot()
    policy2 = SoccerPolicy(robot2)
    far_left_ball = _FakeDetection("soccer_ball", 0.9, x=0, y=150, width=20, height=20)
    policy2.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([far_left_ball]), wall_unknown, hold_toggle=False)
    policy2.reset()
    policy2.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([]), wall_unknown, hold_toggle=False)
    assert robot2.calls[-1][2:] == (SEARCH_SPEED, TURN_MS), (
        "expected a fresh search after reset(), not a coast/prediction carried over from before it", robot2.calls[-1]
    )
    print("  -> reset() cleared tracker, search bias, and both tick counters; no stale state survived")

    print("SELF-TEST PASSED")
