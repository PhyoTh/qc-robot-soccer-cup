import json
import time
from typing import Any, Callable

from arduino.app_utils import Bridge


class ProgramStopped(Exception):
    """Raised when the start/stop button disables the program mid-routine."""


class MiniAutoRobot:
    def __init__(self) -> None:
        self._session_active = False

    def is_running(self) -> bool:
        return bool(self.read_sensors().get("program_enabled"))

    def hold_toggle(self) -> bool:
        """Read-only state of the CAM boot button's 5-second-hold toggle (red/blue LED)."""
        return bool(self.read_sensors().get("hold_toggle"))

    def _require_running(self) -> None:
        if not self.is_running():
            raise ProgramStopped

    def drive(self, command: str, speed: int = 150, ms: int = 500) -> None:
        self._require_running()
        Bridge.call("drive", command, int(speed), int(ms))
        time.sleep(ms / 1000.0 + 0.1)
        self._require_running()

    def stop(self) -> bool:
        return bool(Bridge.call("stop"))

    def read_sensors(self) -> dict[str, Any]:
        raw = Bridge.call("read_sensors")
        return json.loads(raw) if raw else {}

    def servo(self, angle: int) -> None:
        self._require_running()
        Bridge.call("servo", int(angle))

    def buzz(self) -> bool:
        return bool(Bridge.call("buzz"))

    def led(self, on: bool) -> bool:
        return bool(Bridge.call("led", bool(on)))

    def drive_raw(self, m0: int, m1: int, m2: int, m3: int, ms: int = 500) -> None:
        self._require_running()
        Bridge.call("drive_raw", int(m0), int(m1), int(m2), int(m3), int(ms))

    def health(self) -> dict[str, Any]:
        raw = Bridge.call("health")
        return json.loads(raw) if raw else {}

    def run_program(self, routine: Callable[[], None]) -> None:
        """Pass this as App.run's user_loop. Runs routine() exactly once each time the
        start button enables the program, and waits for the next press if a run finishes
        early (routine raised ProgramStopped, e.g. via drive()/servo()) or completes."""
        if not self.is_running():
            self._session_active = False
            return
        if self._session_active:
            return
        self._session_active = True
        print("[INFO] program enabled - starting")
        try:
            routine()
            print("[INFO] run complete - press the button again to restart")
        except ProgramStopped:
            print("[INFO] program stopped - press the button again to restart")
        finally:
            self.stop()
            print(f"[INFO] hold_toggle landed on: {'blue' if self.hold_toggle() else 'red'}")
            self._session_active = False
