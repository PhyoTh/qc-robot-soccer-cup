"""
wall_detector.py  -  HSV colour classification for field-side / wall detection.

Tells the robot which END of the field the camera is facing. Combined with
robot.hold_toggle() (False=red team, True=blue team) that gives OWN SIDE vs
OPPONENT SIDE, which is what soccer_policy.py needs to avoid own goals.

Pure OpenCV/numpy HSV thresholding - no ML model or training data involved.

WHY THIS USES CENTROID POSITION, NOT PIXEL AREA (read before retuning):

The original version counted red vs blue pixels over the whole frame and
took whichever had more. That is the wrong operator for THIS field, and it
was measured failing on real camera frames from the venue (Aug 14).

The field's tape is not "red on one end wall, blue on the other" as README
originally described. It runs along BOTH SIDE walls, split at the midline:
the red half of the field has red tape down both side walls, the blue half
has blue. So a forward-facing camera almost always sees BOTH colours at
once - the half it is standing in, and the half it is heading toward.

Counting area then measures the WRONG THING. The near half's tape is closer
to the camera, so it occupies far more pixels regardless of which way the
robot faces. Area is a proxy for proximity, and proximity tells you which
half you are STANDING IN - the opposite of the question being asked. On one
venue frame the near wall gave red 18,685px vs blue 5,008px while the robot
was in fact facing the blue goal.

Position is the signal that actually answers it. On a side wall receding
toward the vanishing point, the tape nearer the image CENTRE is farther away
in the world - so it marks the end being approached. Whichever colour's
horizontal centroid sits closer to frame centre is the end we face.

Measured on six labelled venue frames (see samples in the team's captures):
    pixel-area rule   2/5 correct  (one CONFIDENTLY wrong, two UNKNOWN)
    centroid rule     5/5 correct
plus a seventh, badly over-exposed frame the centroid rule also got right,
so it is not merely fitted to one lighting condition.

KNOWN GAP: with the camera facing a SIDE wall (no goal ahead at all) both
colours are visible and the rule still returns an answer rather than
UNKNOWN. Left deliberately un-guarded: a margin threshold cannot separate
that case (its margin was 293px) from the genuine-but-tight
blue-goal-from-red case (28px), and the consequences of being wrong while
facing a side wall are cheap - you push the ball into a wall, or peel off
it. Neither concedes a goal. Do not "fix" this with a margin cutoff without
re-measuring; it will reject the good answer and keep the bad one.

Consumed by python/soccer_policy.py and python/diagnostics.py.
"""
try:
    import cv2
    import numpy as np
    _CV_IMPORT_ERROR = None
except ImportError as exc:  # cv2/numpy only guaranteed on-device, not on a plain laptop
    cv2 = None
    np = None
    _CV_IMPORT_ERROR = exc

# HSV bounds per README's "HSV ranges for the field tape" table (OpenCV hue 0-179).
RED_HUE_RANGES = ((0, 10), (160, 179))
BLUE_HUE_RANGE = (100, 130)

# Vertical slice of the frame the decision is made on, as fractions of frame
# height. The camera is wide-angle: the top of the frame is venue ceiling
# (lights, windows) and the bottom is field floor - neither carries tape.
# The walls sit in a band between them. Measured across the six venue frames,
# anything reasonably wide works (20-60%, 25-55%, 30-50% and 15-65% all
# scored 5/5); only narrow or badly-offset bands lost a case. 20-60% is
# deliberately generous so a small change in camera pitch can't slice the
# walls out of view entirely.
BAND_TOP_FRAC = 0.20
BAND_BOTTOM_FRAC = 0.60


def _require_cv() -> None:
    if _CV_IMPORT_ERROR is not None:
        raise RuntimeError(
            "wall_detector requires opencv-python and numpy on the robot's Linux side. "
            "Install with: pip install opencv-python numpy"
        ) from _CV_IMPORT_ERROR


class WallSideDetector:
    def __init__(
        self,
        sat_min: int = 120,
        val_min: int = 70,
        min_tape_px: int = 250,
        dominance_ratio: float = 6.0,
        band_top_frac: float = BAND_TOP_FRAC,
        band_bottom_frac: float = BAND_BOTTOM_FRAC,
    ) -> None:
        self.sat_min = sat_min
        self.val_min = val_min
        # How many tape pixels of a colour must be present in the band before
        # that colour is considered visible at all. Replaces the old
        # min_coverage_pct area gate, which rejected genuinely-visible tape:
        # on real venue frames the correct colour came in at 0.17%-2.29% of
        # frame, i.e. below or barely above the old 2.0% floor.
        self.min_tape_px = min_tape_px
        # When one colour outnumbers the other by at least this factor, decide
        # on AREA and skip the centroid comparison entirely. Measured on the
        # field Aug 14: parked facing the blue goal, blue read ~1300px at
        # offset ~100px while red read only ~90px of scattered noise that
        # happened to sit near frame centre - and the centroid rule handed the
        # verdict to that noise, flipping RED/BLUE ~20 times while the robot
        # sat perfectly still. Centroid is the right operator only when BOTH
        # colours are genuinely present (the case Victor measured); when one
        # is 6x+ the other, the big one is the real tape and the small one is
        # speckle. This guard is deliberately narrow so it cannot touch the
        # 18,685px-vs-5,008px case (ratio 3.7) that motivated the centroid
        # rule in the first place.
        self.dominance_ratio = dominance_ratio
        self.band_top_frac = band_top_frac
        self.band_bottom_frac = band_bottom_frac

    def _masks(self, hsv):
        lo_sv = np.array([0, self.sat_min, self.val_min])
        hi_sv = np.array([0, 255, 255])

        (r0_lo, r0_hi), (r1_lo, r1_hi) = RED_HUE_RANGES
        lo_sv[0], hi_sv[0] = r0_lo, r0_hi
        red_mask_a = cv2.inRange(hsv, lo_sv, hi_sv)
        lo_sv[0], hi_sv[0] = r1_lo, r1_hi
        red_mask_b = cv2.inRange(hsv, lo_sv, hi_sv)
        red_mask = cv2.bitwise_or(red_mask_a, red_mask_b)

        b_lo, b_hi = BLUE_HUE_RANGE
        lo_sv[0], hi_sv[0] = b_lo, b_hi
        blue_mask = cv2.inRange(hsv, lo_sv, hi_sv)
        return red_mask, blue_mask

    def analyze(self, frame_bgr) -> dict:
        """Decide which end of the field the camera faces. See the module
        docstring for why this compares centroid POSITION rather than pixel
        area - that choice is the whole point of this function."""
        _require_cv()
        empty = {
            "side": "UNKNOWN", "red_pct": 0.0, "blue_pct": 0.0,
            "red_px": 0, "blue_px": 0, "red_offset": None, "blue_offset": None,
        }
        if frame_bgr is None or frame_bgr.size == 0:
            return empty

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[0], hsv.shape[1]
        if h == 0 or w == 0:
            return empty

        red_mask, blue_mask = self._masks(hsv)

        # Restrict the decision to the wall band - ceiling and floor carry no
        # tape and only add noise. Guard against a degenerate band on very
        # short frames (the synthetic self-test frames below are 40px tall).
        y0 = min(int(h * self.band_top_frac), max(h - 1, 0))
        y1 = max(int(h * self.band_bottom_frac), y0 + 1)
        red_band, blue_band = red_mask[y0:y1], blue_mask[y0:y1]

        band_px = red_band.shape[0] * red_band.shape[1]
        red_xs = np.nonzero(red_band)[1]
        blue_xs = np.nonzero(blue_band)[1]
        red_px, blue_px = int(red_xs.size), int(blue_xs.size)

        center_x = w / 2.0
        red_offset = float(abs(red_xs.mean() - center_x)) if red_px else None
        blue_offset = float(abs(blue_xs.mean() - center_x)) if blue_px else None

        red_seen = red_px >= self.min_tape_px
        blue_seen = blue_px >= self.min_tape_px
        if not red_seen and not blue_seen:
            side = "UNKNOWN"
        elif not blue_seen:
            side = "RED"
        elif not red_seen:
            side = "BLUE"
        elif red_px >= blue_px * self.dominance_ratio:
            # Overwhelming area difference - see dominance_ratio in __init__.
            side = "RED"
        elif blue_px >= red_px * self.dominance_ratio:
            side = "BLUE"
        else:
            # Both genuinely present in comparable amounts - the one nearer
            # frame centre is nearer the vanishing point, i.e. the far end,
            # i.e. the one we're facing.
            side = "RED" if red_offset < blue_offset else "BLUE"

        return {
            "side": side,
            "red_pct": 100.0 * red_px / band_px if band_px else 0.0,
            "blue_pct": 100.0 * blue_px / band_px if band_px else 0.0,
            "red_px": red_px,
            "blue_px": blue_px,
            "red_offset": red_offset,
            "blue_offset": blue_offset,
        }

    def classify(self, frame_bgr, team_is_blue: bool) -> dict:
        analysis = self.analyze(frame_bgr)
        team = "BLUE" if team_is_blue else "RED"
        wall = analysis["side"]
        if wall == "UNKNOWN":
            result = "UNKNOWN"
        elif wall == team:
            result = "OWN SIDE"
        else:
            result = "OPPONENT SIDE"
        return {**analysis, "team": team, "wall": wall, "result": result}

    def diagnostic_line(self, analysis: dict) -> str:
        def _off(v):
            return f"{v:5.0f}px" if v is not None else "    --"
        # Offsets are the numbers that actually drive the decision - percentages
        # are shown only as a sanity check on whether any tape was seen at all.
        return (
            f"[WallDetector] red={analysis['red_pct']:5.2f}% off={_off(analysis['red_offset'])}  "
            f"blue={analysis['blue_pct']:5.2f}% off={_off(analysis['blue_offset'])}  "
            f"-> facing {analysis['side']}  (nearer-centre colour wins)"
        )

    def field_line(self, classification: dict) -> str:
        return f"[FIELD] team={classification['team']}  wall={classification['wall']}  -> {classification['result']}"


if __name__ == "__main__":
    _require_cv()

    def _solid_bgr_frame(hue: int, sat: int, val: int, size: int = 40):
        """Build a small synthetic BGR frame that is a solid HSV colour, for a
        camera-free self-test (no venue, no robot, no camera needed tonight)."""
        hsv_pixel = np.uint8([[[hue, sat, val]]])
        bgr_pixel = cv2.cvtColor(hsv_pixel, cv2.COLOR_HSV2BGR)[0, 0]
        frame = np.zeros((size, size, 3), dtype=np.uint8)
        frame[:, :] = bgr_pixel
        return frame

    red_frame = _solid_bgr_frame(hue=5, sat=200, val=200)
    blue_frame = _solid_bgr_frame(hue=115, sat=200, val=200)
    neutral_frame = _solid_bgr_frame(hue=45, sat=40, val=110)  # dull grey/green, low saturation

    detector = WallSideDetector()

    print("[SELF-TEST] red frame:")
    red_analysis = detector.analyze(red_frame)
    print(detector.diagnostic_line(red_analysis))
    red_classification = detector.classify(red_frame, team_is_blue=False)
    print(detector.field_line(red_classification))
    assert red_analysis["side"] == "RED", f"expected RED, got {red_analysis['side']}"
    assert red_classification["result"] == "OWN SIDE", f"expected OWN SIDE, got {red_classification['result']}"

    print("[SELF-TEST] blue frame:")
    blue_analysis = detector.analyze(blue_frame)
    print(detector.diagnostic_line(blue_analysis))
    blue_classification = detector.classify(blue_frame, team_is_blue=False)
    print(detector.field_line(blue_classification))
    assert blue_analysis["side"] == "BLUE", f"expected BLUE, got {blue_analysis['side']}"
    assert blue_classification["result"] == "OPPONENT SIDE", f"expected OPPONENT SIDE, got {blue_classification['result']}"

    print("[SELF-TEST] neutral frame:")
    neutral_analysis = detector.analyze(neutral_frame)
    print(detector.diagnostic_line(neutral_analysis))
    neutral_classification = detector.classify(neutral_frame, team_is_blue=True)
    print(detector.field_line(neutral_classification))
    assert neutral_analysis["side"] == "UNKNOWN", f"expected UNKNOWN, got {neutral_analysis['side']}"
    assert neutral_classification["result"] == "UNKNOWN", f"expected UNKNOWN, got {neutral_classification['result']}"

    # --- The regression test that matters: this is the real venue failure ----
    # Recreates the geometry that broke the old area-counting rule. A LARGE
    # patch of one colour out at the frame edge (the near side wall, beside
    # us) and a SMALL patch of the other near frame centre (the far wall, the
    # end we're heading toward). Area says the big edge patch wins; the truth
    # is the small central one. On the real frame this was red 18,685px at the
    # edge vs blue 5,008px near centre, while the robot faced the blue goal.
    def _near_far_frame(near_hue: int, far_hue: int, w: int = 320, h: int = 240):
        frame = _solid_bgr_frame(hue=45, sat=10, val=90, size=8)[0, 0]  # dull background
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:, :] = frame
        band_y0, band_y1 = int(h * 0.30), int(h * 0.45)
        near = cv2.cvtColor(np.uint8([[[near_hue, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
        far = cv2.cvtColor(np.uint8([[[far_hue, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
        img[band_y0:band_y1, int(w * 0.72):] = near   # big, out at the edge
        img[band_y0:band_y1, int(w * 0.44):int(w * 0.56)] = far  # small, near centre
        return img

    print("[SELF-TEST] REGRESSION: big RED patch at the edge, small BLUE patch near centre")
    frame = _near_far_frame(near_hue=5, far_hue=115)
    analysis = detector.analyze(frame)
    print(detector.diagnostic_line(analysis))
    assert analysis["red_px"] > analysis["blue_px"], (
        "test setup is wrong - red should have MORE pixels, that's the whole point"
    )
    assert analysis["side"] == "BLUE", (
        f"centroid rule must pick the colour nearer frame centre (BLUE) even though "
        f"RED has more pixels ({analysis['red_px']} vs {analysis['blue_px']}) - got {analysis['side']}"
    )
    print(f"  -> BLUE, despite RED having {analysis['red_px']}px vs {analysis['blue_px']}px (area would have said RED)")

    print("[SELF-TEST] REGRESSION mirrored: big BLUE at the edge, small RED near centre")
    frame = _near_far_frame(near_hue=115, far_hue=5)
    analysis = detector.analyze(frame)
    print(detector.diagnostic_line(analysis))
    assert analysis["blue_px"] > analysis["red_px"], "test setup is wrong - blue should have MORE pixels"
    assert analysis["side"] == "RED", (
        f"expected RED (nearer centre) despite BLUE having more pixels - got {analysis['side']}"
    )
    print(f"  -> RED, despite BLUE having {analysis['blue_px']}px vs {analysis['red_px']}px")

    print("[SELF-TEST] NEW REGRESSION: a small speck near centre must NOT outvote a big band of the other colour")
    # Reproduces the failure seen on the field Aug 14: parked facing the blue
    # goal, blue read ~1300px at offset ~100px while red was ~90px of speckle
    # sitting near frame centre. The centroid rule handed the verdict to the
    # speckle and the answer flipped RED/BLUE ~20 times while the robot sat
    # still. Big honest band of blue at the EDGE, tiny red speck at CENTRE.
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    blue_bgr = cv2.cvtColor(np.uint8([[[115, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
    red_bgr = cv2.cvtColor(np.uint8([[[5, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
    frame[int(240 * 0.25):int(240 * 0.50), 250:310] = blue_bgr   # 60x60 = 3600px, far right
    frame[int(240 * 0.30):int(240 * 0.34), 155:170] = red_bgr    # ~9x15 = 135px, dead centre
    analysis = detector.analyze(frame)
    print(detector.diagnostic_line(analysis))
    assert analysis["blue_px"] > analysis["red_px"] * 6, "test setup: blue must dominate by >6x"
    assert analysis["side"] == "BLUE", (
        f"a {analysis['red_px']}px red speck near centre must not outvote {analysis['blue_px']}px "
        f"of real blue tape - got {analysis['side']}. This is the field bug the dominance guard fixes."
    )
    print(f"  -> BLUE ({analysis['blue_px']}px) correctly beat the {analysis['red_px']}px centre speck")

    print("[SELF-TEST] only one colour visible -> that colour, no centroid comparison needed")
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    frame[int(240 * 0.30):int(240 * 0.45), 200:] = cv2.cvtColor(
        np.uint8([[[115, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
    analysis = detector.analyze(frame)
    assert analysis["side"] == "BLUE", f"expected BLUE, got {analysis['side']}"
    assert analysis["red_px"] < detector.min_tape_px, "no red should be visible here"
    print(f"  -> {analysis['side']}  (blue_px={analysis['blue_px']}, red_px={analysis['red_px']})")

    print("[SELF-TEST] tape present but only in the ceiling/floor, outside the wall band -> UNKNOWN")
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    frame[0:int(240 * 0.15), :] = cv2.cvtColor(np.uint8([[[5, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
    analysis = detector.analyze(frame)
    assert analysis["side"] == "UNKNOWN", (
        f"colour above the wall band must be ignored, got {analysis['side']}"
    )
    print(f"  -> UNKNOWN  (red in the ceiling correctly ignored)")

    print("SELF-TEST PASSED")
