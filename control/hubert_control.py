import time
from typing import Iterable, Sequence

import serial

PORT = "/dev/tty.usbmodem206EF1333F082"
#PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
RESET_DELAY_S = 2.0
TARGET_TIMEOUT_S = 30.0


def send_angles_to_robot(
    angles: Iterable[Sequence[float]],
    wait_for_enter: bool = False,
    coordinates: Iterable[Sequence[float]] | None = None,
) -> None:
    """Send each target and wait until all joints reach it."""
    with serial.Serial(PORT, BAUD_RATE, timeout=2) as robot:
        time.sleep(RESET_DELAY_S)
        robot.reset_input_buffer()

        angle_points = list(angles)
        coordinate_points = list(coordinates) if coordinates is not None else None
        if coordinate_points is not None and len(coordinate_points) != len(angle_points):
            raise ValueError("Coordinates and angle points must have the same length")

        for point_number, point in enumerate(angle_points):
            if len(point) != 6:
                raise ValueError("Every angle point must contain 6 values")

            command = ",".join(map(str, point)) + "\n"
            if coordinate_points is not None:
                print(
                    f"Sending point {point_number + 1}/{len(angle_points)}: "
                    f"coordinate {coordinate_points[point_number]}"
                )
            robot.write(command.encode())
            robot.flush()

            deadline = time.monotonic() + TARGET_TIMEOUT_S
            while time.monotonic() < deadline:
                response = robot.readline().decode(errors="ignore").strip()
                if response == "TARGET_REACHED":
                    break
            else:
                raise TimeoutError("Arduino did not report TARGET_REACHED")

            if wait_for_enter and point_number < len(angle_points) - 1:
                input("Target reached. Press Enter to send the next position...")