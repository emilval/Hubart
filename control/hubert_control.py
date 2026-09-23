import time
from typing import Iterable, Sequence

import serial

PORT = "/dev/tty.usbmodem206EF1333F082"
BAUD_RATE = 115200
RESET_DELAY_S = 2.0
POINT_INTERVAL_S = 0.090


def send_angles_to_robot(
    angles: Iterable[Sequence[float]],
    point_interval_s: float = POINT_INTERVAL_S,
) -> None:
    """Send six joint angles per point to all_joints.ino."""
    with serial.Serial(PORT, BAUD_RATE, timeout=2) as robot:
        time.sleep(RESET_DELAY_S)

        for point in angles:
            if len(point) != 6:
                raise ValueError("Every angle point must contain 6 values")

            command = ",".join(map(str, point)) + "\n"
            robot.write(command.encode())
            robot.flush()
            time.sleep(point_interval_s)