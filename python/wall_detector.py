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

# Connected components smaller than this many pixels are discarded from the
# colour masks before anything is counted. This fixes a REAL bug seen on the
# robot: facing the blue end, the verdict alternated between BLUE and RED
# frame to frame.
#
# Cause: the venue's walls are very dark (median brightness ~29 - the camera
# meters for the bright floor), and sensor noise on near-black pixels lands
# near hue 0, which is inside RED_HUE_RANGES. So a frame containing genuinely
# ZERO red tape still produces scattered red specks. Once that speck count
# clears min_tape_px, the centroid comparison starts running against a phantom
# whose centroid is essentially random - and a random centroid beats real tape
# about half the time. Measured on "blue goal from blue.png": clean frame has
# red_px=0 and reads BLUE every time; with mild sensor noise red_px=46 and it
# read RED on 33 of 40 trials, purely from noise.
#
# Filtering by COMPONENT SIZE rather than by morphological opening matters
# here. An open (even a 3x3) erodes the real tape too, and at distance the far
# tape is already fragmented - in "red goal from blue.png" the whole red run is
# 110px split across 14 components, largest only 38px. A 3x3 open erased it and
# flipped that frame to the wrong answer. Noise specks are 1-3px, so a size
# filter separates them cleanly without touching real (if small) tape.
#
# Swept against the labelled venue frames: 8-15 all scored 5/5 clean AND 5/5
# stable under simulated sensor noise. Below 8 the flicker returns; at 20+ real
# distant tape starts being discarded. 10 sits in the middle of that plateau.
# Set to 0 or 1 to disable.
MIN_COMPONENT_PX = 10


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
        min_tape_px: int = 50,
        band_top_frac: float = BAND_TOP_FRAC,
        band_bottom_frac: float = BAND_BOTTOM_FRAC,
        min_component_px: int = MIN_COMPONENT_PX,
    ) -> None:
        self.sat_min = sat_min
        self.val_min = val_min
        # How many tape pixels of a colour must be present in the band before
        # that colour is considered visible at all. Replaces the old
        # min_coverage_pct area gate, which rejected genuinely-visible tape:
        # on real venue frames the correct colour came in at 0.17%-2.29% of
        # frame, i.e. below or barely above the old 2.0% floor.
        self.min_tape_px = min_tape_px
        # Deliberately kept LOW. Raising it to 250 was tried as a way to shut
        # out the Aug 14 speckle flicker and reverted: real distant tape is
        # faint, and in "red goal from blue" the entire red run is only ~110px,
        # so a 250 gate rejects genuine tape and gets that frame wrong. The
        # speckle is dealt with properly by min_component_px below, which
        # removes noise at source instead of raising the bar for everyone.
        #
        # NOTE: an area-dominance shortcut ("if one colour has Nx the pixels,
        # just pick it") was also tried here and REMOVED - do not re-add it. It
        # looks reasonable but breaks the most important case on this field:
        # a robot hugging the red side wall while facing the blue goal sees a
        # huge near-red band and only a small far-blue one, so any area rule
        # confidently reports RED while the robot is in fact facing BLUE -
        # precisely the own-goal error the centroid rule exists to prevent.
        self.band_top_frac = band_top_frac
        self.band_bottom_frac = band_bottom_frac
        self.min_component_px = min_component_px
        # Preallocated HSV bounds - analyze() runs on every tick that reaches
        # the policy's scoring decision, so avoid rebuilding six small numpy
        # arrays per call. Mutated in place by _masks(); not thread-safe, which
        # is fine - the policy loop is single-threaded by construction.
        if np is not None:
            (r0_lo, r0_hi), (r1_lo, r1_hi) = RED_HUE_RANGES
            b_lo, b_hi = BLUE_HUE_RANGE
            self._lo = np.array([0, sat_min, val_min], dtype=np.uint8)
            self._hi = np.array([0, 255, 255], dtype=np.uint8)
            self._hue_bounds = ((r0_lo, r0_hi), (r1_lo, r1_hi), (b_lo, b_hi))
            self._col_idx = None  # lazily sized to frame width, reused after

    def _masks(self, hsv):
        """Red/blue masks for an already-cropped HSV band. Mutates the
        preallocated bound arrays in place rather than rebuilding them."""
        lo, hi = self._lo, self._hi
        (r0_lo, r0_hi), (r1_lo, r1_hi), (b_lo, b_hi) = self._hue_bounds

        lo[0], hi[0] = r0_lo, r0_hi
        red_mask = cv2.inRange(hsv, lo, hi)
        lo[0], hi[0] = r1_lo, r1_hi
        # in-place OR into red_mask - saves allocating a third mask per call
        cv2.bitwise_or(red_mask, cv2.inRange(hsv, lo, hi), dst=red_mask)

        lo[0], hi[0] = b_lo, b_hi
        blue_mask = cv2.inRange(hsv, lo, hi)

        # Drop speckle before anything is counted - see MIN_COMPONENT_PX.
        return self._drop_small(red_mask), self._drop_small(blue_mask)

    def _drop_small(self, mask):
        """Zero out connected components below min_component_px. Sensor noise
        on the venue's near-black walls reads as 1-3px red specks; real tape is
        a contiguous run even when distant and fragmented."""
        if self.min_component_px <= 1:
            return mask
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        if count <= 1:
            return mask
        keep = (stats[:, cv2.CC_STAT_AREA] >= self.min_component_px).astype(np.uint8) * 255
        keep[0] = 0  # component 0 is always the background
        return keep[labels]

    def _column_stats(self, mask, w: int):
        """(pixel count, horizontal centroid) for a 0/255 mask.

        Deliberately avoids np.nonzero(), which materialises one int64 index
        array per matched pixel - on a frame with a lot of tape that is the
        single biggest allocation in this function. Summing down columns
        first collapses the work to O(width) and keeps it inside OpenCV."""
        col = cv2.reduce(mask, 0, cv2.REDUCE_SUM, dtype=cv2.CV_32S).reshape(-1)
        total = int(col.sum())
        if total == 0:
            return 0, None
        if self._col_idx is None or self._col_idx.shape[0] != w:
            self._col_idx = np.arange(w, dtype=np.float64)
        # The 255 scaling cancels in the weighted mean, so no need to divide it out.
        centroid = float(col.dot(self._col_idx) / total)
        return total // 255, centroid

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

        h, w = frame_bgr.shape[0], frame_bgr.shape[1]
        if h == 0 or w == 0:
            return empty

        # Crop to the wall band BEFORE any per-pixel work. Ceiling and floor
        # carry no tape, so converting and thresholding them is pure waste -
        # and with the default 20-60% band that is 60% of the frame skipped
        # in both cvtColor and inRange, the two dominant costs here. Slicing a
        # numpy array is a view, so the crop itself is free.
        # Guard against a degenerate band on very short frames (the synthetic
        # self-test frames below are 40px tall).
        y0 = min(int(h * self.band_top_frac), max(h - 1, 0))
        y1 = max(int(h * self.band_bottom_frac), y0 + 1)
        band_bgr = frame_bgr[y0:y1]

        hsv = cv2.cvtColor(band_bgr, cv2.COLOR_BGR2HSV)
        red_band, blue_band = self._masks(hsv)

        band_px = red_band.shape[0] * red_band.shape[1]
        red_px, red_cx = self._column_stats(red_band, w)
        blue_px, blue_cx = self._column_stats(blue_band, w)

        center_x = w / 2.0
        red_offset = abs(red_cx - center_x) if red_cx is not None else None
        blue_offset = abs(blue_cx - center_x) if blue_cx is not None else None

        red_seen = red_px >= self.min_tape_px
        blue_seen = blue_px >= self.min_tape_px
        if not red_seen and not blue_seen:
            side = "UNKNOWN"
        elif not blue_seen:
            side = "RED"
        elif not red_seen:
            side = "BLUE"
        else:
            # Both genuinely present (both cleared min_tape_px) - the one
            # nearer frame centre is nearer the vanishing point, i.e. the far
            # end, i.e. the one we're facing. Area is NOT usable here however
            # lopsided it looks; see the note in __init__.
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

    # NOTE: a test modelling the Aug 14 speckle as one SOLID 135px block used to
    # sit here and was removed - it was a bad model of the failure. Real sensor
    # noise is scattered 1-3px specks, which min_component_px removes; a solid
    # block is indistinguishable from genuine small tape and SHOULD survive
    # filtering. The scattered-speckle regression further down tests the real
    # thing, including asserting the frame still flips with the filter off.
    blue_bgr = cv2.cvtColor(np.uint8([[[115, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
    red_bgr = cv2.cvtColor(np.uint8([[[5, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]

    print("[SELF-TEST] NEW REGRESSION: hugging the RED side wall while facing the BLUE goal")
    # The case that killed an earlier area-dominance shortcut. Pressed against
    # the red side wall, near-red fills a huge slab of frame while the far blue
    # end is small but near centre. ANY rule that prefers the bigger area says
    # RED here and drives the ball toward our own goal. Only position is right.
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    frame[int(240 * 0.20):int(240 * 0.60), 190:320] = red_bgr   # huge near wall, right side
    frame[int(240 * 0.33):int(240 * 0.41), 150:178] = blue_bgr  # small far end, near centre
    analysis = detector.analyze(frame)
    print(detector.diagnostic_line(analysis))
    assert analysis["red_px"] > analysis["blue_px"] * 6, (
        "test setup: near-red must massively outnumber far-blue, that's the trap"
    )
    assert analysis["side"] == "BLUE", (
        f"facing the BLUE end while hugging the red wall must report BLUE, got {analysis['side']} "
        f"(red={analysis['red_px']}px off={analysis['red_offset']:.0f} vs "
        f"blue={analysis['blue_px']}px off={analysis['blue_offset']:.0f}). If this fails, someone "
        f"re-added an area-dominance shortcut - remove it, it causes own goals."
    )
    print(
        f"  -> BLUE, correctly ignoring {analysis['red_px']}px of near wall vs only "
        f"{analysis['blue_px']}px of far tape ({analysis['red_px'] / max(analysis['blue_px'], 1):.0f}x more red)"
    )

    print("[SELF-TEST] only one colour visible -> that colour, no centroid comparison needed")
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    frame[int(240 * 0.30):int(240 * 0.45), 200:] = cv2.cvtColor(
        np.uint8([[[115, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
    analysis = detector.analyze(frame)
    assert analysis["side"] == "BLUE", f"expected BLUE, got {analysis['side']}"
    assert analysis["red_px"] < detector.min_tape_px, "no red should be visible here"
    print(f"  -> {analysis['side']}  (blue_px={analysis['blue_px']}, red_px={analysis['red_px']})")

    # --- Regression for the flicker seen on the robot (see MIN_COMPONENT_PX) ---
    # Real blue tape as one solid blob, plus scattered single-pixel red specks
    # of the kind sensor noise produces on the venue's near-black walls. The
    # specks are deliberately placed NEARER frame centre than the real tape, so
    # if they are counted at all they WIN the centroid comparison and the
    # verdict flips to RED - which is exactly what the robot was doing.
    print("[SELF-TEST] REGRESSION: scattered red speckle must not outvote a solid blue blob")
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    blue_bgr = cv2.cvtColor(np.uint8([[[115, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
    red_bgr = cv2.cvtColor(np.uint8([[[5, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
    frame[80:110, 230:300] = blue_bgr           # solid real tape, off to one side
    rng = np.random.default_rng(0)
    for _ in range(400):                         # speckle clustered near frame centre
        sy = int(rng.integers(60, 130))
        sx = int(rng.integers(140, 180))
        frame[sy, sx] = red_bgr
    analysis = detector.analyze(frame)
    print(detector.diagnostic_line(analysis))
    assert analysis["side"] == "BLUE", (
        f"speckle outvoted real tape - the robot's BLUE/RED flicker is back. got {analysis['side']}\n"
        f"  red_px={analysis['red_px']} blue_px={analysis['blue_px']}"
    )
    assert analysis["red_px"] < detector.min_tape_px, (
        f"expected the speckle to be filtered out entirely, got red_px={analysis['red_px']}"
    )
    print(f"  -> BLUE, speckle filtered to red_px={analysis['red_px']} (blue_px={analysis['blue_px']})")

    print("[SELF-TEST] ...and with the filter disabled, that same frame DOES flip (proves the test bites)")
    unfiltered = WallSideDetector(min_component_px=0)
    flipped = unfiltered.analyze(frame)
    assert flipped["side"] == "RED", (
        "the regression frame no longer reproduces the bug - it isn't testing anything. "
        f"got {flipped['side']} with red_px={flipped['red_px']}"
    )
    print(f"  -> RED without the filter (red_px={flipped['red_px']}) - the frame genuinely reproduces it")

    print("[SELF-TEST] tape present but only in the ceiling/floor, outside the wall band -> UNKNOWN")
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    frame[0:int(240 * 0.15), :] = cv2.cvtColor(np.uint8([[[5, 220, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
    analysis = detector.analyze(frame)
    assert analysis["side"] == "UNKNOWN", (
        f"colour above the wall band must be ignored, got {analysis['side']}"
    )
    print(f"  -> UNKNOWN  (red in the ceiling correctly ignored)")

    print("SELF-TEST PASSED")
