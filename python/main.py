import time

from arduino.app_utils import App
from robot_client import MiniAutoRobot

robot = MiniAutoRobot()

print(f"health   : {robot.health()}")
print(f"sensors  : {robot.read_sensors()}")

# Track the toggle so we only print when it actually changes.
# Hold the CAM boot button for 5 seconds to switch teams.
_last_toggle = robot.hold_toggle()
print(f"[TEAM] active team: {'BLUE' if _last_toggle else 'RED'}  (hold CAM button 5 s to switch)")


def drive(direction: str, speed: int = 150, ms: int = 500) -> None:
    """Drive and wait for the move to finish before continuing."""
    print(f"  {direction} speed={speed} ms={ms}")
    robot.drive(direction, speed, ms)


def loop() -> None:
    global _last_toggle
    current = robot.hold_toggle()
    if current != _last_toggle:
        _last_toggle = current
        team = "BLUE" if current else "RED"
        print(f"[TEAM] switched to: {team}")

    # --- Move ---
    drive("forward",      speed=150, ms=500)
    drive("backward",     speed=150, ms=500)
    drive("left",         speed=150, ms=500)   # strafe left
    drive("right",        speed=150, ms=500)   # strafe right
    drive("rotate_left",  speed=255, ms=3250)  # spin in place, 255 firmware cap on speed
    time.sleep(0.5)
    drive("rotate_right", speed=255, ms=3250)

    robot.stop()
    time.sleep(0.5)

    # --- Sensors ---
    sensors = robot.read_sensors()
    print(f"ultrasonic : {sensors.get('ultrasonic_cm')} cm")
    print(f"battery    : {sensors.get('battery_mv')} mV")
    print(f"line       : {sensors.get('line_digital')}")

    # --- Extras ---
    robot.led(True)
    robot.servo(90)
    robot.servo(150)
    robot.servo(30)
    robot.servo(90)
    robot.led(False)

    robot.stop()


print("[INFO] waiting for BOOT button to start...")
try:
    App.run(user_loop=lambda: robot.run_program(loop))
finally:
    robot.stop()
