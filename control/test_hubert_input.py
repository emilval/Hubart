"""Examples:

Dry run with joint angles:
    .venv/bin/python control/test_hubert_input.py --angles 0 0 0 0 0 0

Send joint angles to Hubert:
    .venv/bin/python control/test_hubert_input.py --angles 0 0 0 0 0 0 --send

Dry run with a Cartesian position:
    .venv/bin/python control/test_hubert_input.py --coordinate 25 0.05 40

Send a Cartesian position to Hubert:
    .venv/bin/python control/test_hubert_input.py --coordinate 25 0.05 40 --send
"""

import argparse
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from control.hubert_control import send_angles_to_robot
# Import 'chain' so we can use it for visualization
from control.inverse_kinematics import get_servo_angles, chain

FIXED_ANGLES = (0, 0, 0)

def coordinate_to_angles(coordinate: list[float]) -> tuple[float, ...]:
    x, y, z = coordinate
    coordinate = [x, y, z]
    return (*get_servo_angles(coordinate), *FIXED_ANGLES)

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
    ik_radians = None  # We need the raw radians to feed back into the visualizer

    if args.coordinate is not None:
        coordinate = [
            args.coordinate[0],
            args.coordinate[1],
            args.coordinate[2],
        ]
        angles = coordinate_to_angles(coordinate)
        # Calculate raw radians explicitly for the visualizer
        ik_radians = chain.inverse_kinematics(coordinate)
        
        print(f"Coordinate: {coordinate}")
        print(f"IK angles: {angles}")
    else:
        angles = tuple(args.angles)
        # Convert manual servo angles back to radians for the IK chain plot
        # chain expects a 5-element array: [base_link, joint1, joint2, joint3, end_effector]
        ik_radians = [0, np.radians(angles[0]), np.radians(angles[1]), np.radians(angles[2]), 0]
        
        print(f"Joint angles: {angles}")

    print(f"Arduino command: {','.join(map(str, angles))}")

    if args.visualize:
        try:
            import matplotlib.pyplot as plt
            
            fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
            # Plot the chain at the calculated/provided angles
            chain.plot(ik_radians, ax, target=coordinate)
            plt.title("Hubert IK Visualization")
            plt.show()
        except ImportError:
            print("Error: matplotlib is required for visualization. Run: pip install matplotlib")

    if args.send:
        send_angles_to_robot([angles], coordinates=[coordinate] if coordinate else None)
    elif not args.visualize:
        print("Dry run only. Add --send to transmit or --visualize to plot this command.")

if __name__ == "__main__":
    main()