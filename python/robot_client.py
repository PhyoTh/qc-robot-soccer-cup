import json
from typing import Any

from arduino.app_utils import Bridge


class MiniAutoRobot:
    def drive(self, command: str, speed: int = 150, ms: int = 500) -> bool:
        return bool(Bridge.call("drive", command, int(speed), int(ms)))

    def stop(self) -> bool:
        return bool(Bridge.call("stop"))

    def read_sensors(self) -> dict[str, Any]:
        raw = Bridge.call("read_sensors")
        if not raw:
            return {}
        return json.loads(raw)

    def servo(self, angle: int) -> bool:
        return bool(Bridge.call("servo", int(angle)))

    def buzz(self) -> bool:
        return bool(Bridge.call("buzz"))

    def led(self, on: bool) -> bool:
        return bool(Bridge.call("led", bool(on)))

    def drive_raw(self, m0: int, m1: int, m2: int, m3: int, ms: int = 500) -> bool:
        return bool(Bridge.call("drive_raw", int(m0), int(m1), int(m2), int(m3), int(ms)))

    def health(self) -> dict[str, Any]:
        raw = Bridge.call("health")
        if not raw:
            return {}
        return json.loads(raw)
