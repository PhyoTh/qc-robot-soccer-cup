import os
import time

from arduino.app_utils import App

from robot_client import MiniAutoRobot


ROBOT_SPEED = int(os.getenv("ROBOCUP_SPEED", "150"))
PULSE_MS = int(os.getenv("ROBOCUP_PULSE_MS", "700"))
PAUSE_SEC = float(os.getenv("ROBOCUP_PAUSE_SEC", "0.25"))

robot = MiniAutoRobot()
_ran_sequence = False


def run_motion_sequence():
    print("MiniAuto motion sequence starting")
    print(f"health={robot.health()}")
    print(f"sensors={robot.read_sensors()}")

    robot.stop()
    time.sleep(PAUSE_SEC)

    for command in ("forward", "left", "right", "backward"):
        print(f"drive {command} speed={ROBOT_SPEED} ms={PULSE_MS}")
        robot.drive(command, ROBOT_SPEED, PULSE_MS)
        time.sleep((PULSE_MS / 1000.0) + PAUSE_SEC)

    robot.stop()
    print("MiniAuto motion sequence complete")


def loop():
    global _ran_sequence

    if not _ran_sequence:
        run_motion_sequence()
        _ran_sequence = True

    time.sleep(0.5)


App.run(user_loop=loop)
