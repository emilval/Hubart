"""Examples:

Dry run with joint angles:
    .venv/bin/python control/test_hubert_input.py --angles 0 0 0 0 0 0

Send joint angles to Hubert:
    .venv/bin/python control/test_hubert_input.py --angles 0 0 0 0 0 0 --send

Dry run with a Cartesian position:
    .venv/bin/python control/test_hubert_input.py --coordinate 0.20 0.00 0.30

Send a Cartesian position to Hubert:
    .venv/bin/python control/test_hubert_input.py --coordinate 0.20 0.00 0.30 --send
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from control.hubert_control import send_angles_to_robot
from control.inverse_kinematics import chain, get_servo_angles

FIXED_ANGLES = (0, 0, 0)


def coordinate_to_angles(coordinate: list[float]) -> tuple[float, ...]:
    """Convert a Cartesian target into the 6-value command format used by Hubert."""
    return (*get_servo_angles(tuple(coordinate), (0.0, 0.0, 0.0)), *FIXED_ANGLES)


def visualize_chain(coordinate: list[float] | None, angles: tuple[float, ...] | None) -> None:
    """Render the IK chain for a single target or explicit angle set."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("Error: matplotlib is required for visualization. Run: pip install matplotlib")
        return

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

    if coordinate is not None:
        initial = np.zeros(len(chain.links))
        initial[1:4] = np.radians((0.0, 0.0, 0.0))
        ik_radians = chain.inverse_kinematics(
            np.asarray(coordinate, dtype=float),
            initial_position=initial,
            optimizer_budget=1000,
            tol=1e-6,
        )
        chain.plot(ik_radians, ax, target=np.asarray(coordinate, dtype=float))
    elif angles is not None:
        ik_radians = [0.0, np.radians(angles[0]), np.radians(angles[1]), np.radians(angles[2]), 0.0]
        chain.plot(ik_radians, ax, target=None)

    plt.title("Hubert IK Visualization")
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send one test coordinate or one angle set to Hubert."
    )
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument(
        "--coordinate",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Cartesian target in metres",
    )
    inputs.add_argument(
        "--angles",
        nargs=6,
        type=float,
        metavar=("BASE", "SHOULDER", "ELBOW", "NECK", "HEAD", "GRIPPER"),
        help="six relative joint angles for all_joints.ino",
    )
    parser.add_argument(
        "--send",
        action="store_true",
        help="actually send the command to Hubert",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Visualize the robot pose using matplotlib instead of/alongside sending",
    )
    args = parser.parse_args()

    coordinate = None
    angles = None

    if args.coordinate is not None:
        coordinate = [float(v) for v in args.coordinate]
        angles = coordinate_to_angles(coordinate)
        print(f"Coordinate: {coordinate}")
        print(f"IK angles: {angles}")
    else:
        angles = tuple(float(v) for v in args.angles)
        print(f"Joint angles: {angles}")

    print(f"Arduino command: {','.join(map(str, angles))}")

    if args.visualize:
        visualize_chain(coordinate, angles)

    if args.send:
        send_angles_to_robot([angles], coordinates=[coordinate] if coordinate is not None else None)
    elif not args.visualize:
        print("Dry run only. Add --send to transmit or --visualize to plot this command.")


if __name__ == "__main__":
    main()