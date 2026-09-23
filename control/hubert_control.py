import time
from typing import Iterable, Sequence

# =========================================================================
# Serial Port Settings
# =========================================================================
PORT = "/dev/tty.usbmodem206EF1333F082"
BAUD_RATE = 115200
SERIAL_RESET_DELAY_S = 2.0

# The firmware moves one received joint target at a time. A short interval
# keeps nearby Cartesian points flowing without building up a serial backlog.
POINT_INTERVAL_S = 0.015
DRAWING_PLANE_Y_M = 0.355
RETRACTION_Y_TOLERANCE_M = 0.0005

# all_joints.ino expects six user angles. The Cartesian IK path controls the
# first three arm joints; keep the remaining joints at their home offsets.
FIXED_JOINT_ANGLES = (0, 0, 0)

# Temporary explicit test path. Replace this array with an import from another
# file when the real coordinate data is available.
TEST_COORDINATES = [
    [0.20, DRAWING_PLANE_Y_M, 0.20],
    [0.225, DRAWING_PLANE_Y_M, 0.225],
    [0.25, DRAWING_PLANE_Y_M, 0.25],
    [0.275, DRAWING_PLANE_Y_M, 0.275],
    [0.30, DRAWING_PLANE_Y_M, 0.30],
]


def send_coordinate_array_to_robot(
    coordinates: Iterable[Sequence[float]],
    point_interval_s: float = POINT_INTERVAL_S,
) -> None:
    """Stream a supplied array of [x, y, z] coordinates to the robot.

    The first point defines the drawing-plane Y coordinate. Later points whose
    Y value differs by more than RETRACTION_Y_TOLERANCE_M are reported as
    retracted points and are still sent through IK like every other point.
    """
    import serial

    from inverse_kinematics import get_servo_angles

    if point_interval_s < 0:
        raise ValueError("point_interval_s cannot be negative")

    path_points = [
        tuple(float(coordinate) for coordinate in point)
        for point in coordinates
    ]
    if not path_points:
        raise ValueError("coordinates must contain at least one point")
    if any(len(point) != 3 for point in path_points):
        raise ValueError("Every coordinate point must contain exactly 3 values")

    initial_y = path_points[0][1]

    try:
        with serial.Serial(PORT, BAUD_RATE, timeout=2) as ser:
            time.sleep(SERIAL_RESET_DELAY_S)

            for point_number, target_xyz in enumerate(path_points, start=1):
                base, shoulder, elbow = get_servo_angles(target_xyz)
                joint_angles = (base, shoulder, elbow, *FIXED_JOINT_ANGLES)
                command = ",".join(str(angle) for angle in joint_angles) + "\n"
                ser.write(command.encode("utf-8"))
                ser.flush()

                point_state = (
                    "drawing"
                    if abs(target_xyz[1] - initial_y) <= RETRACTION_Y_TOLERANCE_M
                    else "retracted"
                )

                print(
                    f"Point {point_number}/{len(path_points)} "
                    f"[{', '.join(f'{coordinate:.4f}' for coordinate in target_xyz)}] "
                    f"({point_state}) -> {command.strip()}"
                )

                # Do not pause after the final point. This avoids an
                # unnecessary delay while still giving the robot time between
                # adjacent targets.
                if point_number < len(path_points):
                    time.sleep(point_interval_s)

            print("Trajectory transmission complete.")
    except (serial.SerialException, ValueError) as error:
        print(f"Robot control error: {error}")


def send_trajectory_to_robot(
    path: Iterable[Sequence[float]],
    point_interval_s: float = POINT_INTERVAL_S,
) -> None:
    """Backward-compatible alias for sending a Cartesian coordinate array."""
    send_coordinate_array_to_robot(path, point_interval_s)


def send_target_to_robot(target_xyz: Sequence[float]) -> None:
    """Backward-compatible wrapper for sending one Cartesian target."""
    send_trajectory_to_robot([target_xyz])


if __name__ == "__main__":
    send_coordinate_array_to_robot(TEST_COORDINATES)