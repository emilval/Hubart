import time
from typing import Iterable, Sequence

import serial

#PORT = "/dev/tty.usbmodem206EF1333F082"
PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
RESET_DELAY_S = 2.0
TARGET_TIMEOUT_S = 60.0
MAX_POINTS_PER_BATCH = 4


def send_angles_to_robot(
    angles: Iterable[Sequence[float]],
    wait_for_enter: bool = False,
    coordinates: Iterable[Sequence[float]] | None = None,
) -> None:
    """Send a trajectory in small acknowledged batches."""
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

        batch_count = (len(angle_points) + MAX_POINTS_PER_BATCH - 1) // MAX_POINTS_PER_BATCH
        print(
            f"Sending {len(angle_points)} trajectory points in {batch_count} batches "
            f"of at most {MAX_POINTS_PER_BATCH}."
        )

        for batch_start in range(0, len(angle_points), MAX_POINTS_PER_BATCH):
            batch_end = min(batch_start + MAX_POINTS_PER_BATCH, len(angle_points))
            batch_angles = angle_points[batch_start:batch_end]
            payload = ";".join(
                ",".join(f"{float(value):.3f}" for value in point)
                for point in batch_angles
            )
            robot.write(f"B:{payload}\n".encode())
            robot.flush()

            deadline = time.monotonic() + TARGET_TIMEOUT_S
            while time.monotonic() < deadline:
                response = robot.readline().decode(errors="ignore").strip()
                if not response.startswith("BUFFER_DONE"):
                    continue

                try:
                    processed_count = int(response.split(":", 1)[1].split()[0])
                except (IndexError, ValueError) as error:
                    raise RuntimeError(
                        f"Could not parse Arduino batch acknowledgement: {response!r}"
                    ) from error

                expected_count = len(batch_angles)
                if processed_count != expected_count:
                    raise RuntimeError(
                        f"Arduino processed {processed_count}/{expected_count} points "
                        f"in batch {batch_start // MAX_POINTS_PER_BATCH + 1}/{batch_count}"
                    )

                print(f"Completed trajectory batch {batch_start // MAX_POINTS_PER_BATCH + 1}/{batch_count}.")
                break
            else:
                raise TimeoutError(
                    f"Arduino did not complete trajectory batch "
                    f"{batch_start // MAX_POINTS_PER_BATCH + 1}/{batch_count}"
                )

        if wait_for_enter:
            input("Trajectory complete. Press Enter to continue...")