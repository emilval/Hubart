import time
from typing import Iterable, Sequence

import serial

PORT = "/dev/tty.usbmodem206EF1333F082"
BAUD_RATE = 115200
RESET_DELAY_S = 2.0
TARGET_TIMEOUT_S = 30.0


def send_angles_to_robot(
    angles: Iterable[Sequence[float]],
) -> None:
    """Send each target and wait until all joints reach it."""
    with serial.Serial(PORT, BAUD_RATE, timeout=2) as robot:
        time.sleep(RESET_DELAY_S)
        robot.reset_input_buffer()

        for point in angles:
            if len(point) != 6:
                raise ValueError("Every angle point must contain 6 values")

            command = ",".join(map(str, point)) + "\n"
            robot.write(command.encode())
            robot.flush()

            deadline = time.monotonic() + TARGET_TIMEOUT_S
            while time.monotonic() < deadline:
                response = robot.readline().decode(errors="ignore").strip()
                if response == "TARGET_REACHED":
                    break
            else:
                raise TimeoutError("Arduino did not report TARGET_REACHED")