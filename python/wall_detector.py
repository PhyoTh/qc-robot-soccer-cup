"""
wall_detector.py  -  HSV colour classification for field-side / wall detection.

Implements the "Wall / Field-Side Detection" section of README.md (near the
end of the file): the soccer field has red tape on one wall and blue tape on
the other. Classifying which colour dominates a camera frame tells the robot
which wall it is facing, and combined with robot.hold_toggle() (False=red
team, True=blue team) that tells it OWN SIDE vs OPPONENT SIDE.

Pure OpenCV/numpy HSV thresholding - no ML model or training data involved.
Consumed by python/soccer_policy.py (built separately, not present yet).
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


def _require_cv() -> None:
    if _CV_IMPORT_ERROR is not None:
        raise RuntimeError(
            "wall_detector requires opencv-python and numpy on the robot's Linux side. "
            "Install with: pip install opencv-python numpy"
        ) from _CV_IMPORT_ERROR


class WallSideDetector:
    def __init__(self, sat_min: int = 120, val_min: int = 70, min_coverage_pct: float = 2.0) -> None:
        self.sat_min = sat_min
        self.val_min = val_min
        self.min_coverage_pct = min_coverage_pct

    def analyze(self, frame_bgr) -> dict:
        _require_cv()
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        total_px = hsv.shape[0] * hsv.shape[1]
        if total_px == 0:
            return {"side": "UNKNOWN", "red_pct": 0.0, "blue_pct": 0.0}

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

        red_pct = 100.0 * int(np.count_nonzero(red_mask)) / total_px
        blue_pct = 100.0 * int(np.count_nonzero(blue_mask)) / total_px

        if red_pct >= self.min_coverage_pct and red_pct >= blue_pct:
            side = "RED"
        elif blue_pct >= self.min_coverage_pct and blue_pct > red_pct:
            side = "BLUE"
        else:
            side = "UNKNOWN"

        return {"side": side, "red_pct": red_pct, "blue_pct": blue_pct}

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
        return f"[WallDetector] red={analysis['red_pct']:.2f}%  blue={analysis['blue_pct']:.2f}%  side={analysis['side']}"

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

    print("SELF-TEST PASSED")
