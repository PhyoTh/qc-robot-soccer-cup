"""
capture.py  -  image capture using Modulino buttons A / B / C.

Button A (left)   : capture into  soccer_ball/  every 0.5 s while held
Button B (middle) : capture into  robot/        every 0.5 s while held
Button C (right)  : capture into  empty/        every 0.5 s while held

Hold any button to stream captures into that label folder.
Release to stop. All three folders are created automatically.

Captured images are saved under:
  captures/soccer_ball/
  captures/robot/
  captures/empty/
"""
import base64
import io, os, json
import requests
import threading
import time

from pathlib import Path
from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI
from PIL import Image

CAMERA_URL      = "http://192.168.5.1:81/stream"
PROJECT_FOLDER  = Path(__file__).resolve().parent.parent
CAPTURES_FOLDER = PROJECT_FOLDER / "captures"

# Maps Bridge method name -> label folder name (matches sketch.ino)
LABELS = {
    "object_a_count": "soccer_ball",
    "object_b_count": "robot",
    "object_c_count": "empty",
}

# How often to save a frame while a button is held (seconds)
CAPTURE_INTERVAL_SECONDS = 0.5

CAPTURE_JPEG_QUALITY = 92
PREVIEW_JPEG_QUALITY = 85
LOOP_DELAY_SECONDS   = 0.05
BRIDGE_RETRY_DELAY   = 0.25

# Create capture folders
for label in LABELS.values():
    os.makedirs(CAPTURES_FOLDER / label, exist_ok=True)

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

current_image  = None
image_lock     = threading.Lock()
preview_image  = b""
preview_updated_at = 0

camera_response = None
camera_stop     = threading.Event()

# Per-button state: last count seen and last capture timestamp
_prev_counts     = {method: None  for method in LABELS}
_last_capture_at = {method: 0.0   for method in LABELS}
_capture_counts  = {method: 0     for method in LABELS}

# ---------------------------------------------------------------------------
# Camera helpers
# ---------------------------------------------------------------------------

def turn_frame_into_jpeg(frame, quality):
    buffer = io.BytesIO()
    frame.save(buffer, "JPEG", quality=quality)
    return buffer.getvalue()


def camera_reader():
    global current_image, camera_response
    while not camera_stop.is_set():
        buffer = b""
        try:
            print(f"[INFO] connecting to camera: {CAMERA_URL}")
            camera_response = requests.get(CAMERA_URL, stream=True, timeout=(5, None))
            camera_response.raise_for_status()
            print(f"[INFO] camera connected: {CAMERA_URL}")
            for chunk in camera_response.iter_content(chunk_size=1024):
                if camera_stop.is_set():
                    break
                buffer += chunk
                while True:
                    start = buffer.find(b"\xff\xd8")
                    end   = buffer.find(b"\xff\xd9", start + 2)
                    if start == -1 or end == -1:
                        break
                    jpg    = buffer[start:end + 2]
                    buffer = buffer[end + 2:]
                    try:
                        frame = Image.open(io.BytesIO(jpg)).convert("RGB")
                        with image_lock:
                            current_image = frame
                    except Exception as e:
                        print(f"[WARN] skipped bad MJPEG frame: {e}")
        except requests.exceptions.RequestException as e:
            print(f"[WARN] camera stream interrupted: {e}")
        finally:
            if camera_response is not None:
                camera_response.close()
        if not camera_stop.is_set():
            print("[INFO] reconnecting to camera in 1 second")
            time.sleep(1)


def start_camera():
    threading.Thread(target=camera_reader, daemon=True).start()


def stop_camera():
    camera_stop.set()
    if camera_response is not None:
        camera_response.close()


def copy_current_image():
    with image_lock:
        return None if current_image is None else current_image.copy()


def update_browser_preview(frame):
    global preview_image, preview_updated_at
    with image_lock:
        preview_image      = turn_frame_into_jpeg(frame, PREVIEW_JPEG_QUALITY)
        preview_updated_at = int(time.time() * 1000)


def get_preview_image():
    with image_lock:
        image_bytes = preview_image
    if not image_bytes:
        return {"image": "", "status": "waiting for camera"}
    return {"image": base64.b64encode(image_bytes).decode("ascii")}


# ---------------------------------------------------------------------------
# Capture helper
# ---------------------------------------------------------------------------

def save_capture(frame, method):
    label    = LABELS[method]
    folder   = CAPTURES_FOLDER / label
    count    = _capture_counts[method]
    filename = f"{label}_{int(time.time() * 1000)}_{count:04d}.jpg"
    path     = folder / filename
    frame.save(path, "JPEG", quality=CAPTURE_JPEG_QUALITY)
    _capture_counts[method] += 1
    _last_capture_at[method] = time.monotonic()
    print(f"[CAPTURE] {label} #{count:04d}  -> {filename}")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def loop():
    frame = copy_current_image()
    if frame is not None:
        update_browser_preview(frame)

    # Read button counts from sketch
    try:
        counts = {method: int(Bridge.call(method)) for method in LABELS}
    except Exception as err:
        print(f"[WARN] Bridge unavailable: {err}")
        time.sleep(BRIDGE_RETRY_DELAY)
        return

    now = time.monotonic()

    for method, count in counts.items():
        prev = _prev_counts[method]

        # First loop — seed counts, don't capture
        if prev is None:
            _prev_counts[method] = count
            continue

        # Button is being held: count keeps incrementing each press (300 ms debounce in sketch)
        # Capture every CAPTURE_INTERVAL_SECONDS regardless of exact count delta
        if count > prev:
            _prev_counts[method] = count
            if frame is None:
                print(f"[WARN] no camera frame yet for {LABELS[method]}")
            elif now - _last_capture_at[method] >= CAPTURE_INTERVAL_SECONDS:
                save_capture(frame, method)

    time.sleep(LOOP_DELAY_SECONDS)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

print(f"[INFO] saving captures under {CAPTURES_FOLDER}")
print(f"[INFO] A={LABELS['object_a_count']}  B={LABELS['object_b_count']}  C={LABELS['object_c_count']}")
print(f"[INFO] hold a button to capture every {CAPTURE_INTERVAL_SECONDS}s into that label folder")

ui = WebUI(assets_dir_path=PROJECT_FOLDER)
ui.expose_api("GET", "/preview", get_preview_image)

start_camera()

try:
    App.run(user_loop=loop)
finally:
    stop_camera()
