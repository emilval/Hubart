import argparse
import importlib.util
from pathlib import Path

from control.hubert_control import send_angles_to_robot
from control.inverse_kinematics import get_servo_angles
from preprocessing.image_pathing import create_robot_paths


IMAGE_PATH = Path(__file__).resolve().parent / "images" / "google_simple_house_0.png"
MAPPING_PATH = Path(__file__).resolve().parent / "2d_to_3d.py"
FIXED_ANGLES = (0, 0, 0)
COORDINATE_DECIMAL_PLACES = 4


def map_image_to_3d(image):
	module_spec = importlib.util.spec_from_file_location("mapping", MAPPING_PATH)
	mapping = importlib.util.module_from_spec(module_spec)
	module_spec.loader.exec_module(mapping)
	return mapping.map_image_to_3d(image)


def coordinates_to_angles(coordinates):
	return [
		(*get_servo_angles(point), *FIXED_ANGLES)
		for stroke in coordinates
		for point in stroke
	]


def main(send_to_robot: bool) -> None:

	image = create_robot_paths(IMAGE_PATH)
	coordinates = [
		[
			(point[0], round(float(point[1]), COORDINATE_DECIMAL_PLACES),
			 round(float(point[2]), COORDINATE_DECIMAL_PLACES))
			for point in stroke
		]
		for stroke in map_image_to_3d(image)
	]
	angles = coordinates_to_angles(coordinates)
	flat_coordinates = [
		point
		for stroke in coordinates
		for point in stroke
	]

	print(f"Image points: {sum(len(stroke) for stroke in image)}")
	print(f"3D coordinates: {sum(len(stroke) for stroke in coordinates)}")
	print(f"Angle commands: {len(angles)}")
	print("Target positions and commands:")
	for point, angle_command in zip(flat_coordinates, angles):
		print(f"  {point} -> {angle_command}")

	if send_to_robot:
		send_angles_to_robot(angles)
	else:
		print("Dry run only. Add --send to control Hubert.")


if __name__ == "__main__":
	parser = argparse.ArgumentParser(
		description="Convert google_simple_house into Hubert drawing commands."
	)
	parser.add_argument(
		"--send",
		action="store_true",
		help="send the mapped trajectory to Hubert via serial",
	)
	args = parser.parse_args()
	main(args.send)

# Quick single-target tests used during setup:
#
# .venv/bin/python main_temp.py --quick-coordinate 0.355 0.15 0.30 --send
# .venv/bin/python main_temp.py --quick-angles 0 -20 40 0 0 0 --send
