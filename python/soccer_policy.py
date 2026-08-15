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

8. THIS POLICY NEVER CALLS robot.stop() - AND MUST NOT. The firmware's stop
   RPC does not merely halt the motors, it also sets program_enabled=false
   (see rpcStop in sketch/sketch.ino), which ends the whole run: the caller's
   `while robot.is_running()` loop exits and a human has to press the BOOT
   button again to resume. The developer guide flags this too - stop() is a
   session-ending fail-safe and finally-cleanup call, not a control-flow tool.

   An earlier version of this file called stop() on a missing frame, a stale
   frame, an empty frame, or an inference error. All four are TRANSIENT, and
   over a Wi-Fi MJPEG stream competing with ~30 other robot APs in the room,
   a dropped frame during a 5-minute match is close to certain. So the robot
   would shut itself off mid-match on the first hiccup and sit dead until
   someone re-pressed BOOT. Caught by a teammate before the tournament.

   Simply returning is already the correct safe response: every drive() this
   policy issues is a bounded 150-250ms pulse that the firmware auto-stops on
   its own timer, so a skipped tick coasts to a halt within one pulse and
   play resumes the moment frames return. Session-ending stop() belongs only
   in the caller's finally block (see main.py).

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
# Per-class confidence floors. MEASURED AT THE VENUE (Aug 14), not guessed:
# with the robot parked facing the red goal, the goal class scored 0.52-0.72
# (mean ~0.57) across 54 consecutive frames. A single 0.6 bar therefore threw
# away most true goal sightings, which is exactly what made goal detection
# look broken on the field - the model saw it, the threshold discarded it.
#
# Ball and robot keep the stricter 0.6: the ball class was the strongest in
# training (~100%) so it does not need the help, and a false ball chase is
# worse than a missed one. Goal gets 0.45, comfortably under the observed
# spread but still well above noise.
#
# NOTE ei_runner.ObjectDetector applies its OWN floor before the policy sees
# anything - it must be constructed with a min_confidence at or below the
# lowest value here or these thresholds can never fire. See main.py.
MIN_CONFIDENCE_BY_LABEL = {
    # Ball also measured at the venue: a ball at 4-6% of frame width (i.e.
    # genuinely far away) scored 0.50-0.72. At a 0.6 bar roughly 40% of true
    # sightings were dropped, which on a small fast field is the difference
    # between tracking the ball and repeatedly re-acquiring it.
    #
    # The false-positive risk is low and this is measured too, not assumed:
    # across 88 consecutive frames with no ball in view, the model produced
    # ZERO spurious soccer_ball detections even at a 0.3 display floor.
    "soccer_ball": 0.45,
    "robot": 0.6,
    "goal": 0.45,
}
MIN_CONFIDENCE = 0.6  # fallback for any label not listed above

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

# How many consecutive ticks to keep rotating the SAME way before reversing.
# This exists because the original code flipped direction every single tick,
# which looked like a sensible "don't spin one way forever" guard but actually
# meant the robot oscillated around one heading - observed on the field as
# jittering in place rather than searching. It never pointed anywhere new, so
# it could not find a ball that wasn't already in front of it.
#
# Sweeping in blocks fixes it: 6 ticks x 200ms of continuous rotation is a
# real arc, then it reverses and sweeps back across the other side. The
# alternation still prevents spinning endlessly in one direction.
#
# SET GENEROUSLY ON PURPOSE. Nobody has measured how many degrees one 200ms
# pulse at SEARCH_SPEED actually turns this chassis, and the failure modes are
# lopsided: too FEW ticks sweeps a narrow wedge, reverses, and re-scans the
# same wedge forever - a ball behind the robot is never found, which looks
# almost as broken as the jitter this replaced. Too MANY just means it rotates
# past the ball and comes back around on the next pass, costing a second.
# 15 x 200ms = 3s of continuous rotation, which should comfortably exceed a
# full circle. Lower it only after watching a real sweep and seeing it
# overshoot badly.
SEARCH_SWEEP_TICKS = 15

# --- Goal-scored heuristic (drives the celebration) -------------------------
# There is no score sensor, so "did we score?" has to be inferred. The signal
# used is: we were pushing a CLOSE ball at a POSITIVELY CONFIRMED opponent
# end, and then the ball disappeared. A ball that vanishes while being driven
# into the opponent's goal mouth most likely went in.
#
# This is a heuristic and it WILL sometimes fire on an occlusion or a ball
# that squirted sideways. That is an accepted trade: the cost of a false
# celebration is a couple of wasted seconds, and the routine is bounded and
# self-stopping. The cost of missing a real goal is no celebration at all,
# which for the Redemption Cup's "Best Goal Celebration" is the entire
# scoring criterion. Bias toward firing.
#
# Both numbers are unvalidated - tune on the field. If it celebrates when the
# ball merely rolls out of view, raise SCORING_PUSH_MIN_SIZE_FRAC (demand the
# ball be closer) or lower SCORING_LOST_WITHIN_TICKS (demand it vanish sooner
# after the push).
SCORING_PUSH_MIN_SIZE_FRAC = 0.12  # ball must fill >=12% of frame width to count as a scoring push
SCORING_LOST_WITHIN_TICKS = 4      # ball must vanish within this many ticks of that push

# --- Kickoff opening move ---------------------------------------------------
# At kickoff the geometry is KNOWN, which is rare and worth exploiting: the
# ball sits on the centre dot, and our robot starts behind the black line in
# the RIGHT-HAND corner in front of our own goal. Strafing left lines us up
# with the goal-to-goal axis (and the ball); driving forward then closes on
# the ball down that line.
#
# Why bother instead of just letting the normal chase run: the model is
# weakest on small distant objects (measured 0.50-0.72 on a ball at only 4-6%
# of frame width), so at kickoff range detection is unreliable. Physically
# closing the distance turns a hard perception problem into an easy one, and
# whoever reaches the ball first controls the match. This is open-loop by
# design - it does not wait to SEE the ball before moving, because waiting is
# the thing that loses the race.
#
# It yields early on either of two conditions (see _run_opening): the ball is
# already detected and roughly centred (chase it properly instead), or an
# opponent looms large in frame (their kickoff burst is on a collision course
# with ours - hand over to the normal policy, which jukes rather than rams;
# tipping an opponent is a Yellow Card).
#
# EVERY VALUE HERE IS UNVALIDATED - the correct strafe duration depends on
# exactly where the referee places the robot and how fast these motors move.
# Tune it on the field: run it, watch where the robot ends up, adjust.
# Set OPENING_SEQUENCE = [] to disable the opening entirely.
# MEASURED FIELD GEOMETRY (given by the team, Aug 14):
#   - strafing sideways ~6 inches puts the robot in front of the goal
#   - from there the ball sits ~15 inches directly ahead
#
# WHICH WAY IS SIDEWAYS depends on the field: on some the robot starts in the
# RIGHT corner (strafe left to centre up), on others the LEFT corner (strafe
# right). The referee's placement decides it, so this is set per match, not
# baked in - see set_opening_corner() and main.py's START_CORNER env var.
# Getting it backwards drives AWAY from the goal into the corner, so check it
# every round the same way you check the team colour.
OPENING_START_CORNER = "right"  # "right" -> strafe left; "left" -> strafe right
OPENING_SIDEWAYS_INCHES = 6.0
OPENING_FORWARD_INCHES = 15.0

# Speeds used for the opening burst. Higher = faster to the ball but less
# accurate distance-per-ms; these are the values the MS_PER_INCH numbers
# below must be calibrated AT.
OPENING_STRAFE_SPEED = 200
OPENING_FORWARD_SPEED = 220

# CALIBRATE THESE ON THE FIELD - run: python3 python/calibrate_motion.py
# Milliseconds of drive time per inch travelled, at the speeds above. The
# defaults are placeholders derived from nothing; mecanum strafing is
# typically slower per-inch than driving forward, which is why they differ.
# Getting these right is the single highest-value tuning step for the
# opening, because everything below is computed from them.
# MEASURED on the field Aug 14 with calibrate_motion.py, at the speeds above.
STRAFE_MS_PER_INCH = 100.0
FORWARD_MS_PER_INCH = 100.0

# The forward burst is split into chunks of at most this many milliseconds,
# with a full Acquire->Infer cycle between each one. At 100 ms/inch the
# approach is ~1100ms, and driving that as a single blocking pulse would mean
# a full second of not looking - straight past the ball if it has rolled off
# the expected line. Chunking trades a little speed (drive() adds ~100ms of
# settle per call) for the ability to spot the ball mid-approach and hand
# over to the real closed-loop chase. 300ms is roughly 3 inches of travel
# between looks.
OPENING_CHUNK_MS = 300

# --- Wall unstick during search ---------------------------------------------
# Reported from the field: while searching, the robot can end up parked
# against a side wall and stay there. Rotating in place cannot wedge it, but
# it also cannot fix it - from hard against a wall most of the frame is wall,
# so there is very little field in view for the ball to appear in, and the
# search can burn the rest of the match looking at plywood.
#
# The ultrasonic sensor is dead on this chassis, so "am I against a wall?"
# has to come from the camera. The signal used is total tape coverage: the
# side walls carry tape along their length, so when the frame is mostly tape
# the wall is filling the view, which only happens up close. Combined red+blue
# coverage above this fraction of the wall band counts as nose-to-wall.
#
# Deliberately only applied while SEARCHING. During ball chase a big tape
# reading is normal and expected (the ball is often near a wall), and
# reversing there would abandon the ball.
WALL_STUCK_COVERAGE_PCT = 30.0
# Only after the search has already failed for a while - a brief glimpse of
# wall mid-sweep is not stuck, it is just part of turning around.
WALL_STUCK_AFTER_TICKS = 6
WALL_BACKUP_SPEED = 140
WALL_BACKUP_MS = 350

# Stop short of the ball rather than driving through where it sits: the
# normal closed-loop chase should make the final approach, and overshooting
# risks knocking the ball somewhere random before we ever see it.
OPENING_FORWARD_STOP_SHORT_INCHES = 4.0

# One extra forward pulse appended to the burst, added after watching it live:
# it was stopping just shy of the ball rather than making contact. Kept as a
# separate tack-on rather than folded into the inches above so it stays easy
# to drop back out if it turns out to overshoot on a different field.
OPENING_EXTRA_FORWARD_MS = 200

def _chunked(direction: str, speed: int, total_ms: int):
    """Split one long move into OPENING_CHUNK_MS-sized steps.

    Each step is a separate policy tick, so a fresh camera frame is inferred
    between them - that is what stops the opening from driving blind past a
    ball that isn't exactly where we assumed.
    """
    steps = []
    remaining = max(int(total_ms), 0)
    while remaining > 0:
        piece = min(remaining, OPENING_CHUNK_MS)
        steps.append((direction, speed, piece))
        remaining -= piece
    return steps


def _build_opening(corner: str):
    """Compute the opening burst for a given starting corner.

    Starting in the RIGHT corner means strafing LEFT to reach the goal line,
    and vice versa - so the sideways step is mirrored while the forward step
    is unchanged.
    """
    sideways = "left" if corner == "right" else "right"
    forward_inches = max(OPENING_FORWARD_INCHES - OPENING_FORWARD_STOP_SHORT_INCHES, 1)
    return (
        # Strafe off the corner, onto the goal-to-goal line.
        _chunked(sideways, OPENING_STRAFE_SPEED, OPENING_SIDEWAYS_INCHES * STRAFE_MS_PER_INCH)
        # Then advance down that line toward the ball, looking between chunks.
        + _chunked("forward", OPENING_FORWARD_SPEED, forward_inches * FORWARD_MS_PER_INCH)
        # Plus one more nudge to actually reach the ball - see above.
        + _chunked("forward", OPENING_FORWARD_SPEED, OPENING_EXTRA_FORWARD_MS)
    )


OPENING_SEQUENCE = _build_opening(OPENING_START_CORNER)


def set_opening_corner(corner: str) -> None:
    """Point the opening at the corner the referee actually placed us in.

    Call before the match (main.py does this from the START_CORNER env var).
    Rebuilds OPENING_SEQUENCE in place so already-constructed policies, which
    read the module-level list each tick, pick the change up immediately.
    """
    global OPENING_START_CORNER, OPENING_SEQUENCE
    corner = corner.strip().lower()
    if corner not in ("left", "right"):
        raise ValueError(f"corner must be 'left' or 'right', got {corner!r}")
    OPENING_START_CORNER = corner
    OPENING_SEQUENCE[:] = _build_opening(corner)
# Abort the opening if a "robot" detection is at least this wide - an opponent
# charging the same spot. Bbox-only on purpose: the ultrasonic sensor is dead
# on this chassis, so proximity has to come from the camera.
OPENING_ABORT_ON_OPPONENT_WIDTH_FRAC = 0.30
# NOTE: the opening also yields the instant the ball is detected AT ALL, at
# any offset - see _run_opening. No threshold for that on purpose: this burst
# exists only to cover the window where the ball is too far to see, so any
# real sighting makes closed-loop chasing the better move.

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

    def __init__(self, robot: "MiniAutoRobot", on_goal_scored=None) -> None:
        """on_goal_scored: optional zero-arg callback fired when the policy
        believes it just scored (see _maybe_goal_scored). Used to trigger the
        Redemption Cup celebration. Left None during normal bracket play -
        celebrating mid-match burns clock for no points."""
        self.robot = robot
        self.on_goal_scored = on_goal_scored
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
        self._search_sweep_ticks = 0  # see SEARCH_SWEEP_TICKS
        self._scan_turn_left_next = True  # alternates the possession-safe "peek" turn
        self._possession_scan_ticks = 0
        self._consecutive_push_ticks = 0  # see DEWEDGE_PUSH_TICKS
        self._last_goal_side: Optional[str] = None  # see GOAL_MEMORY_TICKS
        self._last_goal_side_age = 0
        self._ticks_since_scoring_push: Optional[int] = None  # see _maybe_goal_scored
        # Kickoff opening. Reset per session on purpose: after a Yellow Card
        # both robots go back to starting position, so the opening applies
        # again on the restart exactly as it did at kickoff.
        self._opening_step = 0

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

        # 2) Never drive on missing or stale vision - but do NOT call
        # robot.stop() here. See design note 8: stop() ends the whole session.
        # Simply returning is already a safe no-op, because every drive() this
        # policy issues is a bounded 150-250ms pulse that the FIRMWARE
        # auto-stops on its own timer. Skipping a tick therefore coasts to a
        # halt within one pulse and resumes the instant frames come back.
        if frame_bgr is None:
            print("[WARN] soccer_policy: no camera frame this tick - skipping (motors auto-stop)")
            return
        if frame_ts is None:
            frame_ts = now
        elif (now - frame_ts) > FRAME_STALE_SEC:
            print(f"[WARN] soccer_policy: frame is {now - frame_ts:.2f}s stale - skipping tick")
            return

        frame_h, frame_w = frame_bgr.shape[0], frame_bgr.shape[1]
        if frame_w <= 0 or frame_h <= 0:
            print("[WARN] soccer_policy: empty frame - skipping tick")
            return

        # 3) Infer, then Validate against our own confidence bar.
        try:
            detections = detector.infer(frame_bgr)
        except Exception as exc:  # noqa: BLE001 - one bad frame must not crash the match loop
            print(f"[WARN] soccer_policy: inference failed ({exc}) - skipping tick")
            return

        opponent = self._above_threshold(detector.best(detections, "robot"))
        raw_ball = self._above_threshold(detector.best(detections, "soccer_ball"))
        track_state = self._tracker.update(raw_ball)

        # Age the goal-scored arm window (see _maybe_goal_scored).
        if self._ticks_since_scoring_push is not None:
            self._ticks_since_scoring_push += 1

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

        # 4.5) --- KICKOFF OPENING -----------------------------------------
        # Runs AFTER the opponent-contact tiers above so safety always wins,
        # and BEFORE ball chase because at kickoff the ball is usually too far
        # to detect reliably - the whole point is to close that distance.
        if self._opening_step < len(OPENING_SEQUENCE):
            if self._run_opening(raw_ball, opponent, frame_w):
                return

        # 5) --- BALL CHASE: tracking / coasting / lost ---------------------
        if track_state == "lost":
            self._possession_scan_ticks = 0
            self._consecutive_push_ticks = 0
            # Check BEFORE clearing goal memory - the check reads it.
            self._maybe_goal_scored()
            self._last_goal_side = None  # ball itself is gone, not just occluded - don't trust old goal memory
            if self._search_turn_left_next is None:
                self._search_turn_left_next = self._tracker.last_known_side_left(frame_w)
                self._search_sweep_ticks = 0
            direction = "rotate_left" if self._search_turn_left_next else "rotate_right"
            # Hold the same direction for a whole sweep before reversing. See
            # SEARCH_SWEEP_TICKS - flipping every tick just jitters in place.
            self._search_sweep_ticks += 1
            if self._search_sweep_ticks >= SEARCH_SWEEP_TICKS:
                self._search_turn_left_next = not self._search_turn_left_next
                self._search_sweep_ticks = 0

            ticks_lost = self._tracker.ticks_since_seen

            # Wall unstick: parked against a side wall, rotating in place just
            # surveys plywood - there is barely any field in frame for the
            # ball to appear in. Back off first, then carry on searching from
            # somewhere the sweep can actually see something.
            if ticks_lost > WALL_STUCK_AFTER_TICKS:
                try:
                    coverage = wall.analyze(frame_bgr)
                    tape_pct = coverage.get("red_pct", 0.0) + coverage.get("blue_pct", 0.0)
                except Exception:  # noqa: BLE001 - never let a vision hiccup break search
                    tape_pct = 0.0
                if tape_pct >= WALL_STUCK_COVERAGE_PCT:
                    print(
                        f"[POLICY] wall fills {tape_pct:.0f}% of view after {ticks_lost} lost ticks "
                        f"- backing off before searching further"
                    )
                    self.robot.drive("backward", speed=WALL_BACKUP_SPEED, ms=WALL_BACKUP_MS)
                    return

            # Escalating search (design note 5): once genuinely lost for a
            # while (past the coast window, then past this extra grace
            # period too), mix in an occasional strafe instead of only
            # rotating in place forever, to cover more of the frame.
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
            # Arm goal detection: we are pushing a CLOSE ball at a CONFIRMED
            # opponent end. If the ball vanishes shortly after this, the most
            # likely explanation is that it went in. See _maybe_goal_scored.
            if size_frac >= SCORING_PUSH_MIN_SIZE_FRAC:
                self._ticks_since_scoring_push = 0
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

    def _run_opening(self, raw_ball, opponent, frame_w: float) -> bool:
        """Execute one step of the kickoff opening. Returns True if it acted
        (caller should return), False to fall through to normal play.

        Yields early - abandoning the rest of the sequence - when continuing
        would be worse than playing normally. See OPENING_SEQUENCE.
        """
        # An opponent filling the frame means their kickoff burst is heading
        # into ours. Hand over: the normal policy jukes around contact rather
        # than driving through it, and tipping an opponent is a Yellow Card.
        if opponent is not None and (opponent.width / frame_w) >= OPENING_ABORT_ON_OPPONENT_WIDTH_FRAC:
            print(
                f"[OPENING] aborted at step {self._opening_step + 1}/{len(OPENING_SEQUENCE)} - "
                f"opponent fills {opponent.width / frame_w:.0%} of frame, playing it normally"
            )
            self._opening_step = len(OPENING_SEQUENCE)
            return False

        # We can SEE the ball - abandon the opening immediately. The entire
        # reason this open-loop burst exists is that the ball is usually too
        # far to detect at kickoff; the moment that stops being true, a real
        # closed-loop chase is strictly better than continuing to guess.
        if raw_ball is not None:
            offset = raw_ball.center_x - frame_w / 2.0
            print(
                f"[OPENING] ball detected (off={offset:+.0f}px) at step "
                f"{self._opening_step + 1}/{len(OPENING_SEQUENCE)} - switching to normal chase"
            )
            self._opening_step = len(OPENING_SEQUENCE)
            return False

        direction, speed, ms = OPENING_SEQUENCE[self._opening_step]
        self._opening_step += 1
        print(
            f"[OPENING] step {self._opening_step}/{len(OPENING_SEQUENCE)}: "
            f"{direction} speed={speed} ms={ms}"
        )
        self.robot.drive(direction, speed=speed, ms=ms)
        return True

    def _maybe_goal_scored(self) -> None:
        """Fire on_goal_scored if the ball vanished right after we drove it,
        close, at a confirmed opponent end. See the SCORING_* constants for
        why this is deliberately biased toward firing.

        Called from the ball-lost branch only, and always disarms afterwards
        so one goal cannot trigger repeated celebrations.
        """
        armed = self._ticks_since_scoring_push
        self._ticks_since_scoring_push = None  # disarm regardless of outcome
        if armed is None or armed > SCORING_LOST_WITHIN_TICKS:
            return
        if self.on_goal_scored is None:
            print(f"[POLICY] GOAL likely scored (ball vanished {armed} tick(s) after a close push)")
            return
        print(f"[POLICY] GOAL! (ball vanished {armed} tick(s) after a close push) - celebrating")
        try:
            self.on_goal_scored()
        except Exception as exc:  # noqa: BLE001 - a failed celebration must never end the match
            print(f"[WARN] celebration raised {exc!r} - continuing play")

    @staticmethod
    def _above_threshold(detection: Optional["Detection"]) -> Optional["Detection"]:
        if detection is None:
            return None
        floor = MIN_CONFIDENCE_BY_LABEL.get(detection.label, MIN_CONFIDENCE)
        if detection.confidence < floor:
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

    print("[SELF-TEST] NEW: per-class confidence floors match what was measured at the venue")
    # Real sightings measured Aug 14: goal 0.52-0.72, distant ball 0.50-0.72.
    # A single 0.6 bar discarded most goals and ~40% of balls - that is what
    # made detection look broken on the field. Both floors are now 0.45.
    _mid_goal = _FakeDetection("goal", 0.51, x=100, y=20, width=120, height=60)
    _mid_ball = _FakeDetection("soccer_ball", 0.51, x=150, y=150, width=20, height=20)
    _weak_ball = _FakeDetection("soccer_ball", 0.30, x=150, y=150, width=20, height=20)
    _mid_robot = _FakeDetection("robot", 0.51, x=20, y=20, width=130, height=100)
    assert SoccerPolicy._above_threshold(_mid_goal) is not None, "0.51 goal must pass - real-world range"
    assert SoccerPolicy._above_threshold(_mid_ball) is not None, "0.51 ball must pass - real-world range"
    assert SoccerPolicy._above_threshold(_weak_ball) is None, "0.30 ball is noise, must still be rejected"
    assert SoccerPolicy._above_threshold(_mid_robot) is None, (
        "robot stays strict at 0.6 - a false opponent triggers juke/backoff and surrenders the ball"
    )
    print(
        f"  -> goal={MIN_CONFIDENCE_BY_LABEL['goal']}  ball={MIN_CONFIDENCE_BY_LABEL['soccer_ball']}  "
        f"robot={MIN_CONFIDENCE_BY_LABEL['robot']}"
    )

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

    print("[SELF-TEST] NEW: search SWEEPS in one direction, it does not jitter tick-to-tick")
    # Observed on the field: the robot "jittered" instead of searching. Cause
    # was flipping rotation direction every single tick, so it oscillated
    # around one heading and never pointed anywhere new - it could only ever
    # find a ball that was already in front of it.
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    policy._opening_step = len(OPENING_SEQUENCE)
    nothing = _FakeDetector([])
    for _ in range(SEARCH_SWEEP_TICKS):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, nothing, wall_unknown, hold_toggle=False)
    first_sweep = [c[1] for c in robot.calls]
    assert len(set(first_sweep)) == 1, (
        f"a sweep must hold ONE direction for {SEARCH_SWEEP_TICKS} ticks, got {first_sweep}"
    )
    # Next tick must reverse, so the sweep covers the other side too.
    policy.decide_and_act(FRAME, ENABLED_SENSORS, nothing, wall_unknown, hold_toggle=False)
    assert robot.calls[-1][1] != first_sweep[0], (
        f"after {SEARCH_SWEEP_TICKS} ticks it must reverse, got {robot.calls[-1][1]} again"
    )
    print(
        f"  -> {SEARCH_SWEEP_TICKS}x {first_sweep[0]} "
        f"({SEARCH_SWEEP_TICKS * TURN_MS}ms of continuous rotation), then reversed to {robot.calls[-1][1]}"
    )

    print("[SELF-TEST] NEW: mid-sweep, the instant the ball appears it stops searching and chases")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    policy._opening_step = len(OPENING_SEQUENCE)
    nothing = _FakeDetector([])
    for _ in range(3):  # part-way through a sweep, still searching
        policy.decide_and_act(FRAME, ENABLED_SENSORS, nothing, wall_unknown, hold_toggle=False)
    assert robot.calls[-1][2:] == (SEARCH_SPEED, TURN_MS), f"should be searching, got {robot.calls[-1]}"
    # Ball comes into view off to the right - must abandon the sweep and aim at it.
    right_ball = _FakeDetection("soccer_ball", 0.9, x=290, y=150, width=20, height=20)  # centre_x=300
    policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([right_ball]), wall_unknown, hold_toggle=False)
    assert robot.calls[-1] == ("drive", "rotate_right", APPROACH_SPEED, TURN_MS), (
        f"expected an aiming turn TOWARD the ball at chase speed, got {robot.calls[-1]}"
    )
    # ...and then close on it once centred.
    centred = _FakeDetection("soccer_ball", 0.9, x=150, y=150, width=40, height=40)
    _goal_ahead = _FakeDetection("goal", 0.9, x=100, y=20, width=120, height=60)
    policy.decide_and_act(
        FRAME, ENABLED_SENSORS, _FakeDetector([centred, _goal_ahead]),
        _FakeWallDetector("OPPONENT SIDE"), hold_toggle=False,
    )
    assert robot.calls[-1][1] == "forward", f"expected a push once centred, got {robot.calls[-1]}"
    print(f"  -> search -> aim ({robot.calls[-2][1]}) -> push ({robot.calls[-1][1]}) with no gap")

    print("[SELF-TEST] no ball detected at all (never seen) -> search, arbitrary default side")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    # Skip past the kickoff opening - this test is about SEARCH alternation.
    # Without this it silently measures the opening's steps instead (which
    # also differ from each other) and stops testing what it claims to.
    policy._opening_step = len(OPENING_SEQUENCE)
    detector = _FakeDetector([])
    policy.decide_and_act(FRAME, ENABLED_SENSORS, detector, wall_unknown, hold_toggle=False)
    policy.decide_and_act(FRAME, ENABLED_SENSORS, detector, wall_unknown, hold_toggle=False)
    first_dir, second_dir = robot.calls[0][1], robot.calls[1][1]
    assert first_dir.startswith("rotate") and second_dir.startswith("rotate"), (
        f"expected two search rotations, got {robot.calls}"
    )
    # Consecutive ticks now hold the SAME direction - that is the sweep fix.
    # Reversing every tick was the jitter bug; the reversal is asserted in the
    # dedicated sweep test above.
    assert first_dir == second_dir, (
        f"consecutive search ticks must sweep the same way, not flip - got {robot.calls}"
    )
    print(f"  -> {first_dir} held across consecutive ticks (sweeping, not jittering)")

    print("[SELF-TEST] program disabled -> no action at all")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    disabled_sensors = {"program_enabled": False, "ultrasonic_mm": -1}
    policy.decide_and_act(FRAME, disabled_sensors, _FakeDetector([]), wall_unknown, hold_toggle=False)
    assert robot.calls == [], robot.calls

    print("[SELF-TEST] NEW: kickoff opening runs its sequence, then hands over to normal play")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    empty_det = _FakeDetector([])
    for i, (direction, speed, ms) in enumerate(OPENING_SEQUENCE):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, empty_det, wall_unknown, hold_toggle=False)
        assert robot.calls[-1] == ("drive", direction, speed, ms), (
            f"opening step {i + 1} should be {direction}, got {robot.calls[-1]}"
        )
    # Sequence exhausted -> normal policy (no ball anywhere -> search)
    policy.decide_and_act(FRAME, ENABLED_SENSORS, empty_det, wall_unknown, hold_toggle=False)
    assert robot.calls[-1][1].startswith("rotate"), (
        f"after the opening the normal search should take over, got {robot.calls[-1]}"
    )
    print(f"  -> ran {len(OPENING_SEQUENCE)} opening step(s), then normal search")

    print("[SELF-TEST] NEW: backs off a wall that fills the view during a prolonged search")

    class _WallHeavyDetector:
        """Wall detector stand-in reporting the frame is mostly tape - what
        the real one returns with the robot's nose against a side wall."""

        def analyze(self, frame_bgr):
            return {"side": "RED", "red_pct": 40.0, "blue_pct": 2.0}

        def classify(self, frame_bgr, team_is_blue):
            return {"side": "RED", "red_pct": 40.0, "blue_pct": 2.0,
                    "team": "RED", "wall": "RED", "result": "OWN SIDE"}

        def field_line(self, classification):
            return "[FIELD] (self-test) wall-heavy"

    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    policy._opening_step = len(OPENING_SEQUENCE)  # not what this test is about
    wall_heavy = _WallHeavyDetector()
    empty = _FakeDetector([])
    # Early ticks: a bit of wall in view is normal mid-sweep, keep rotating.
    for _ in range(WALL_STUCK_AFTER_TICKS):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, empty, wall_heavy, hold_toggle=False)
        assert robot.calls[-1][1].startswith("rotate"), (
            f"should still be sweeping this early, got {robot.calls[-1]}"
        )
    # Past the grace period with the wall still filling the frame -> back off.
    policy.decide_and_act(FRAME, ENABLED_SENSORS, empty, wall_heavy, hold_toggle=False)
    assert robot.calls[-1] == ("drive", "backward", WALL_BACKUP_SPEED, WALL_BACKUP_MS), (
        f"expected a back-off once wedged against the wall, got {robot.calls[-1]}"
    )
    print(f"  -> swept {WALL_STUCK_AFTER_TICKS} ticks, then backed off: {robot.calls[-1]}")

    print("[SELF-TEST] NEW: a wall in view does NOT interrupt an active ball chase")
    # Tape filling the frame is normal when the ball is near a wall - reversing
    # there would abandon the ball, so the unstick must be search-only.
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    policy._opening_step = len(OPENING_SEQUENCE)
    side_ball = _FakeDetection("soccer_ball", 0.9, x=0, y=150, width=20, height=20)
    for _ in range(WALL_STUCK_AFTER_TICKS + 3):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([side_ball]), wall_heavy, hold_toggle=False)
        assert robot.calls[-1][1] != "backward", (
            f"must keep chasing the ball even with a wall in frame, got {robot.calls[-1]}"
        )
    print(f"  -> kept chasing across {WALL_STUCK_AFTER_TICKS + 3} ticks: {robot.calls[-1]}")

    print("[SELF-TEST] NEW: opening spots the ball MID-APPROACH and stops driving blind")
    # The forward burst is chunked precisely so a fresh frame is inferred
    # between pieces. Drive a few chunks with nothing in view, then reveal the
    # ball part-way through: it must abandon the rest of the burst that same
    # tick rather than finishing a pre-planned drive past it.
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    empty_det = _FakeDetector([])
    for _ in range(3):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, empty_det, wall_unknown, hold_toggle=False)
    steps_used = policy._opening_step
    assert steps_used < len(OPENING_SEQUENCE), "test needs steps remaining to prove early exit"
    side_ball = _FakeDetection("soccer_ball", 0.9, x=20, y=150, width=20, height=20)  # well off to the left
    policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([side_ball]), wall_unknown, hold_toggle=False)
    assert policy._opening_step >= len(OPENING_SEQUENCE), (
        f"seeing the ball must abandon the remaining {len(OPENING_SEQUENCE) - steps_used} opening step(s)"
    )
    assert robot.calls[-1][1] == "rotate_left", (
        f"expected the normal chase to turn toward the ball, got {robot.calls[-1]}"
    )
    print(
        f"  -> after {steps_used}/{len(OPENING_SEQUENCE)} chunks the ball appeared; "
        f"burst abandoned and it turned to chase ({robot.calls[-1][1]})"
    )

    print("[SELF-TEST] NEW: opening ABORTS if an opponent is charging the same spot (collision risk)")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    big_opponent = _FakeDetection("robot", 0.9, x=40, y=20, width=140, height=120)  # 44% of frame
    policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([big_opponent]), wall_unknown, hold_toggle=False)
    assert not [c for c in robot.calls if c[1] in ("left", "forward") and c[3] > 400], (
        f"must not blindly burst into a looming opponent, got {robot.calls}"
    )
    assert policy._opening_step >= len(OPENING_SEQUENCE), "opening should be abandoned, not merely paused"
    print(f"  -> aborted the burst and played normally: {robot.calls[-1]}")

    print("[SELF-TEST] NEW: opening yields early once the ball is already centred")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    centred_ball = _FakeDetection("soccer_ball", 0.9, x=150, y=150, width=20, height=20)  # centre_x=160
    policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([centred_ball]), wall_unknown, hold_toggle=False)
    assert policy._opening_step >= len(OPENING_SEQUENCE), "should abandon the opening once the ball is in view"
    # Compare against the opening's exact (direction, speed, ms) - checking the
    # direction alone gives a false alarm, since the possession-safe peek also
    # strafes "left" but at a different speed/duration.
    assert tuple(robot.calls[-1][1:]) not in {tuple(s) for s in OPENING_SEQUENCE}, (
        f"expected normal play, not an opening step: {robot.calls[-1]}"
    )
    print(f"  -> skipped straight to normal chase: {robot.calls[-1]}")

    print("[SELF-TEST] NEW: reset() re-arms the opening (Yellow Card sends both robots back to start)")
    robot = _FakeRobot()
    policy = SoccerPolicy(robot)
    policy.decide_and_act(FRAME, ENABLED_SENSORS, empty_det, wall_unknown, hold_toggle=False)
    assert policy._opening_step == 1, policy._opening_step
    policy.reset()
    assert policy._opening_step == 0, "after a card restart the opening must run again from the top"
    print("  -> opening re-armed after reset()")

    print("[SELF-TEST] NEW: goal-scored fires when a close ball vanishes at a confirmed opponent end")
    fired = []
    robot = _FakeRobot()
    policy = SoccerPolicy(robot, on_goal_scored=lambda: fired.append(True))
    close_ball = _FakeDetection("soccer_ball", 0.9, x=140, y=150, width=60, height=60)  # 19% of frame
    opp_goal = _FakeDetection("goal", 0.9, x=100, y=20, width=120, height=60)
    wall_opp = _FakeWallDetector("OPPONENT SIDE")
    # Push at a confirmed opponent end with the ball close...
    policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([close_ball, opp_goal]), wall_opp, hold_toggle=False)
    assert robot.calls[-1][1] == "forward", robot.calls
    # ...then the ball vanishes. Coast window first, then genuinely lost.
    for _ in range(COAST_TICKS + 1):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([]), wall_opp, hold_toggle=False)
    assert fired, "expected the goal callback to fire when the ball vanished right after a close push"
    print(f"  -> callback fired once ({len(fired)}x total)")

    print("[SELF-TEST] NEW: goal-scored does NOT fire when the ball simply drifts away far from goal")
    fired = []
    robot = _FakeRobot()
    policy = SoccerPolicy(robot, on_goal_scored=lambda: fired.append(True))
    far_ball = _FakeDetection("soccer_ball", 0.9, x=152, y=150, width=15, height=15)  # 5% - too far to be a scoring push
    policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([far_ball, opp_goal]), wall_opp, hold_toggle=False)
    for _ in range(COAST_TICKS + 1):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([]), wall_opp, hold_toggle=False)
    assert not fired, "a distant ball going out of view is not a goal - must not celebrate"
    print("  -> correctly stayed silent (ball was only 5% of frame, not a scoring push)")

    print("[SELF-TEST] NEW: a celebration that raises must not end the match")
    robot = _FakeRobot()
    def _boom():
        raise RuntimeError("celebration exploded")
    policy = SoccerPolicy(robot, on_goal_scored=_boom)
    policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([close_ball, opp_goal]), wall_opp, hold_toggle=False)
    for _ in range(COAST_TICKS + 1):
        policy.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([]), wall_opp, hold_toggle=False)  # must not raise
    print("  -> exception swallowed, play continued")

    print("[SELF-TEST] CRITICAL: a transient camera/inference failure must NEVER call stop()")
    # stop() sets program_enabled=false in firmware, ending the run until a
    # human re-presses BOOT. Over a Wi-Fi MJPEG stream sharing the room with
    # ~30 other robot APs, a dropped frame in a 5-minute match is near certain
    # - so calling stop() here means the robot shuts itself off mid-match on
    # the first hiccup. All four transient paths must simply skip the tick and
    # let the firmware's own bounded-pulse timer coast the motors to a halt.
    class _ExplodingDetector:
        def infer(self, frame_bgr):
            raise RuntimeError("simulated inference failure")

        def best(self, detections, label):
            return None

    for label, frame, det, ts in (
        ("missing frame", None, _FakeDetector([]), None),
        ("empty frame", _FakeFrame(0, 0), _FakeDetector([]), None),
        ("stale frame", FRAME, _FakeDetector([]), time.monotonic() - (FRAME_STALE_SEC + 5)),
        ("inference error", FRAME, _ExplodingDetector(), None),
    ):
        robot = _FakeRobot()
        policy = SoccerPolicy(robot)
        policy.decide_and_act(frame, ENABLED_SENSORS, det, wall_unknown, hold_toggle=False, frame_ts=ts)
        assert ("stop",) not in robot.calls, (
            f"{label}: called stop(), which ends the run and requires a human BOOT press to recover. "
            f"Return instead - bounded drive pulses auto-stop in firmware. Got {robot.calls}"
        )
        assert robot.calls == [], f"{label}: must not drive on bad vision either, got {robot.calls}"
    print("  -> all four transient failures skip the tick without ending the session")

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
    # reset() correctly re-arms the kickoff opening (asserted separately above);
    # skip it here so this assertion measures ball-tracker state as intended.
    policy2._opening_step = len(OPENING_SEQUENCE)
    policy2.decide_and_act(FRAME, ENABLED_SENSORS, _FakeDetector([]), wall_unknown, hold_toggle=False)
    assert robot2.calls[-1][2:] == (SEARCH_SPEED, TURN_MS), (
        "expected a fresh search after reset(), not a coast/prediction carried over from before it", robot2.calls[-1]
    )
    print("  -> reset() cleared tracker, search bias, and both tick counters; no stale state survived")

    print("SELF-TEST PASSED")
