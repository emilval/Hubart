import argparse
import importlib.util
from pathlib import Path

from control.hubert_control import send_angles_to_robot
from control.inverse_kinematics import get_servo_angles
from preprocessing.image_pathing import create_robot_paths


IMAGE_PATH = Path(__file__).resolve().parent / "images" / "square.png"
MAPPING_PATH = Path(__file__).resolve().parent / "preprocessing" / "2d_to_3d.py"
FIXED_ANGLES = (0, 0, 0)
COORDINATE_DECIMAL_PLACES = 4
LIFT_X_OFFSET = -0.05
LIFT_STEP_COUNT = 10


def add_stroke_transition_points(coordinates):
	"""Insert a lifted move and re-entry between consecutive strokes.

	The resulting structure remains a list of strokes so the inverse kinematics
	continues to receive the previous point in order and does not break continuity.
	"""
	prepared = []

	for stroke_index, stroke in enumerate(coordinates):
		if not stroke:
			continue

		prepared.append(stroke)

		if stroke_index >= len(coordinates) - 1:
			continue

		next_stroke = coordinates[stroke_index + 1]
		if not next_stroke:
			continue

		last_x, last_y, last_z = stroke[-1]
		pullback_points = []
		for step in range(1, LIFT_STEP_COUNT + 1):
			lift_x = round(
				last_x + (LIFT_X_OFFSET * step / LIFT_STEP_COUNT),
				COORDINATE_DECIMAL_PLACES,
			)
			pullback_points.append((lift_x, last_y, last_z))

		next_start_x, next_start_y, next_start_z = next_stroke[0]
		next_hover_x = round(next_start_x + LIFT_X_OFFSET, COORDINATE_DECIMAL_PLACES)
		transition_points = [
			*pullback_points,
			(next_hover_x, next_start_y, next_start_z),
		]

		for step in range(1, LIFT_STEP_COUNT + 1):
			reentry_x = round(
				next_start_x + (LIFT_X_OFFSET * (1 - step / LIFT_STEP_COUNT)),
				COORDINATE_DECIMAL_PLACES,
			)
			transition_points.append((reentry_x, next_start_y, next_start_z))

		prepared.append(transition_points)

	return prepared


def map_image_to_3d(image):
	module_spec = importlib.util.spec_from_file_location("mapping", MAPPING_PATH)
	mapping = importlib.util.module_from_spec(module_spec)
	module_spec.loader.exec_module(mapping)
	return mapping.map_image_to_3d(image)


def coordinates_to_angles(coordinates):
	angle_commands = []
	previous_angles = (0, 0, 0)  
	for stroke in coordinates:
		for point in stroke:
			servo_angles = get_servo_angles(point, previous_angles)
			angle_commands.append((*servo_angles, *FIXED_ANGLES))
			previous_angles = servo_angles
	return angle_commands


def main(send_to_robot: bool) -> None:

	image = create_robot_paths(IMAGE_PATH)
	print("\nIMAGE PATHING - 2D COORDINATES:")
	for stroke_number, stroke in enumerate(image, start=1):
		for point_number, point in enumerate(stroke, start=1):
			print(f"  stroke {stroke_number}, point {point_number}: {tuple(point)}")

	coordinates = [
		[
			(
				point[0],
				round(float(point[1]), COORDINATE_DECIMAL_PLACES),
				round(float(point[2]), COORDINATE_DECIMAL_PLACES),
			)
			for point in stroke
		]
		for stroke in map_image_to_3d(image)
	]
	print(coordinates)
	coordinates = add_stroke_transition_points(coordinates)

	print("\n2D TO 3D - TRANSLATED COORDINATES:")
	for stroke_number, stroke in enumerate(coordinates, start=1):
		for point_number, point in enumerate(stroke, start=1):
			print(f"  stroke {stroke_number}, point {point_number}: {point}")


	angles = coordinates_to_angles(coordinates)
	flat_coordinates = [
		point
		for stroke in coordinates
		for point in stroke
	]
	print("\nHUBERT CONTROL - ANGLES:")
	for point_number, angle_command in enumerate(angles, start=1):
		print(f"  point {point_number}: {angle_command}")

	print(f"Image points: {sum(len(stroke) for stroke in image)}")
	print(f"3D coordinates: {sum(len(stroke) for stroke in coordinates)}")
	print(f"Angle commands: {len(angles)}")
	
	if send_to_robot:
		send_angles_to_robot(
			angles,
			wait_for_enter=False,
			coordinates=flat_coordinates,
		)
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
