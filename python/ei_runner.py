"""
ei_runner.py - thin wrapper around the Edge Impulse Linux Python SDK.

Wraps the FOMO object-detection model documented in README.md's "Model
Import" section (classes: goal, robot, soccer_ball; input 96x96 RGB, resize
mode "squash") so python/soccer_policy.py (built separately by a teammate,
not present in this repo yet) can ask "what do I see" without touching the
Edge Impulse SDK directly.

No .eim model file exists in this repo today. It is trained and exported
from Edge Impulse Studio (Deployment tab, target Linux/aarch64) once
labeling/training is good enough - probably tonight or tomorrow on-site -
then placed under models/, e.g. models/soccer-linux-aarch64.eim (see
README.md's "Model Import" section). .eim files are intentionally
gitignored and must never be expected to exist ahead of that export.

Both the "edge_impulse_linux" PyPI package and the exported model file are
optional at import time, so this module stays importable (and
py_compile-able) on a plain laptop with neither installed.
"""
import os
import sys
from dataclasses import dataclass
from typing import Optional

try:
    import numpy as np
except ImportError:
    np = None

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from edge_impulse_linux.image import ImageImpulseRunner
except ImportError:
    ImageImpulseRunner = None


@dataclass
class Detection:
    label: str
    confidence: float
    x: int
    y: int
    width: int
    height: int

    @property
    def center_x(self) -> float:
        return self.x + self.width / 2

    @property
    def center_y(self) -> float:
        return self.y + self.height / 2


class ModelUnavailable(RuntimeError):
    """Raised when the Edge Impulse SDK or the exported .eim model can't be used yet."""


class ObjectDetector:
    """Runs the exported FOMO model (goal / robot / soccer_ball) against camera frames."""

    def __init__(self, model_path: str, min_confidence: float = 0.6) -> None:
        self.model_path = model_path
        self.min_confidence = min_confidence
        self.labels: list[str] = []
        self._runner = None

    def __enter__(self) -> "ObjectDetector":
        # Check the SDK and the model file before ever touching the runner, so a
        # missing export today fails fast with an actionable message instead of a
        # confusing traceback from inside the SDK.
        if ImageImpulseRunner is None:
            raise ModelUnavailable(
                "edge_impulse_linux is not installed - run: pip install edge_impulse_linux"
            )
        if not os.path.isfile(self.model_path):
            raise ModelUnavailable(
                f"model file not found: {self.model_path}\n"
                "Export the trained model from Edge Impulse Studio (Deployment tab, "
                "target the Linux/aarch64 .eim build), then place the downloaded file "
                "at this path, e.g. models/soccer-linux-aarch64.eim "
                "(see README.md's 'Model Import' section)."
            )
        self._runner = ImageImpulseRunner(self.model_path)
        model_info = self._runner.init()
        self.labels = model_info["model_parameters"]["labels"]
        print(f"[INFO] ei_runner: loaded model - labels={self.labels}")
        return self

    def __exit__(self, *exc) -> None:
        if self._runner is not None:
            self._runner.stop()
            self._runner = None

    def infer(self, frame_bgr) -> list[Detection]:
        """frame_bgr: HxWx3 numpy array in BGR order (as produced by cv2/PIL->cv2)."""
        if self._runner is None:
            raise ModelUnavailable("ObjectDetector.infer() called outside a 'with' block")
        if cv2 is None:
            raise ModelUnavailable("opencv-python is not installed - run: pip install opencv-python")

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        features, _cropped = self._runner.get_features_from_image_auto_studio_settings(frame_rgb)
        res = self._runner.classify(features)

        boxes = res["result"]["bounding_boxes"]
        detections = [
            Detection(
                label=box["label"],
                confidence=float(box["value"]),
                x=int(box["x"]),
                y=int(box["y"]),
                width=int(box["width"]),
                height=int(box["height"]),
            )
            for box in boxes
            if box["value"] >= self.min_confidence
        ]
        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections

    def best(self, detections: list[Detection], label: str) -> Optional[Detection]:
        matches = [d for d in detections if d.label == label]
        return max(matches, key=lambda d: d.confidence) if matches else None


if __name__ == "__main__":
    model_path = sys.argv[1] if len(sys.argv) > 1 else None

    if not model_path or not os.path.isfile(model_path):
        print(
            "[ei_runner] no .eim model available in this environment - "
            "skipping live self-test (expected until the model is exported)"
        )
        sys.exit(0)

    if ImageImpulseRunner is None:
        print("[ei_runner] edge_impulse_linux not installed - skipping live self-test")
        sys.exit(0)

    if cv2 is None or np is None:
        print("[ei_runner] cv2/numpy not installed - skipping live self-test")
        sys.exit(0)

    print(f"[INFO] ei_runner: smoke-testing model at {model_path}")
    blank_bgr = np.zeros((96, 96, 3), dtype=np.uint8)
    with ObjectDetector(model_path) as detector:
        found = detector.infer(blank_bgr)
        print(f"[INFO] ei_runner: {len(found)} detection(s) found")
        for d in found:
            print(f"  {d.label} conf={d.confidence:.2f} xy=({d.x},{d.y}) wh=({d.width},{d.height})")
