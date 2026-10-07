import time
from typing import Iterable, Sequence

import serial

#PORT = "/dev/tty.usbmodem206EF1333F082"
PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
RESET_DELAY_S = 2.0
TARGET_TIMEOUT_S = 60.0


def send_angles_to_robot(
    angles: Iterable[Sequence[float]],
    wait_for_enter: bool = False,
    coordinates: Iterable[Sequence[float]] | None = None,
) -> None:
    """Send a buffered trajectory of several joint targets in one serial packet."""
    with serial.Serial(PORT, BAUD_RATE, timeout=2) as robot:
        time.sleep(RESET_DELAY_S)
        robot.reset_input_buffer()

        angle_points = list(angles)
        coordinate_points = list(coordinates) if coordinates is not None else None
        if coordinate_points is not None and len(coordinate_points) != len(angle_points):
            raise ValueError("Coordinates and angle points must have the same length")

        if not angle_points:
            return

        for point in angle_points:
            if len(point) != 6:
                raise ValueError("Every angle point must contain 6 values")

        if coordinate_points is not None:
            print(
                f"Sending buffered trajectory: {len(angle_points)} points "
                f"for coordinates={coordinate_points[0]} ... {coordinate_points[-1]}"
            )

        payload = ";".join(
            ",".join(f"{float(value):.3f}" for value in point)
            for point in angle_points
        )
        command = f"B:{payload}\n".encode()
        robot.write(command)
        robot.flush()

        deadline = time.monotonic() + TARGET_TIMEOUT_S
        while time.monotonic() < deadline:
            response = robot.readline().decode(errors="ignore").strip()
            if response.startswith("BUFFER_DONE"):
                break
        else:
            raise TimeoutError("Arduino did not report BUFFER_DONE")

        if wait_for_enter:
            input("Trajectory buffer complete. Press Enter to continue...")