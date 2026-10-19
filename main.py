import argparse
from preprocessing.image_generators import GoogleImageGenerator
from preprocessing.image_pathing import create_robot_paths
from preprocessing.audio_capture import AudioListener
import numpy as np
from pathlib import Path
import re
import argparse
import importlib.util
from pathlib import Path
import numpy as np
from control.hubert_control import parse_robot_status, query_robot_status, send_angles_to_robot
from control.inverse_kinematics import get_servo_angles, forward_kinematics
from preprocessing.image_pathing import create_robot_paths
from preprocessing.map_2d_to_3d import add_page_tilt

MAPPING_PATH = Path(__file__).resolve().parent / "preprocessing" / "map_2d_to_3d.py"
FIXED_ANGLES = (0, 0, 0)
COORDINATE_DECIMAL_PLACES = 4
LIFT_X_OFFSET = -0.05
LIFT_STEP_COUNT = 10



#PROMPT_INJECTION = " It should simple black and white line art with simple lines. Minamalistic with no details."
PROMPT_INJECTION = " Minimalist line art icon, black and white, simple bold outlines, clean white background, minimalist 2d, no shading, no color, thin black stroke, no background at all"
def get_unique_image_path(prompt: str, base_dir: Path = Path("./images"), max_length: int = 50) -> Path:
    """
    Creates a unique snake_case file path from a prompt. 
    Appends _0, _1, etc., if files already exist.
    """
    # Ensure the directory exists
    base_dir.mkdir(parents=True, exist_ok=True)
    
    # Convert to lowercase and replace whitespace with underscores
    clean_prompt = prompt.lower().strip()
    clean_prompt = re.sub(r'\s+', '_', clean_prompt)
    
    # Remove any non-alphanumeric characters (keep underscores)
    clean_prompt = re.sub(r'[^a-z0-9_]', '', clean_prompt)
    
    # Truncate to avoid excessively long filenames
    clean_prompt = clean_prompt[:max_length].strip('_')
    
    # Fallback just in case the prompt was entirely special characters
    if not clean_prompt:
        clean_prompt = "generated_image"
        
    # Iterate to find the next available number (_0, _1, ...)
    counter = 0
    while True:
        file_path = base_dir / f"{clean_prompt}_{counter}.png"
        if not file_path.exists():
            return file_path
        counter += 1


def get_image_path(args, image_generator, audio_listener) -> Path:
    if args.image:
        return Path(args.image)

    description = args.description

    if not description:
        description = get_audio_description(audio_listener)

    out_path = get_unique_image_path(description)

    full_prompt = description + PROMPT_INJECTION    

    print(f"Full generation prompt: {description}")

    image_generator.generate_image(
        prompt=full_prompt,
        output_path=out_path
    )

    return out_path

def get_audio_description(audio_listener) -> str:
    while True:
        print("Please state your prompt:")
        description = audio_listener.listen_for_audio()

        feedback = input(
            f'Is this description correct "{description}"? (y/n): '
        )

        if feedback.lower() == "y":
            return description

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


def coordinates_to_angles(coordinates, starting_angles):
	angle_commands = []
	previous_angles = starting_angles
	for stroke in coordinates:
		for point in stroke:
			servo_angles = get_servo_angles(point, previous_angles)
			angle_commands.append((*servo_angles, *FIXED_ANGLES))
			previous_angles = servo_angles
	return angle_commands


def draw_image(image_path: Path, send_to_robot: bool = False, starting_angles: tuple = None) -> None:

	image = create_robot_paths(image_path)
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
	coordinates = add_page_tilt(coordinates, tilt_angle_degrees=0)
	starting_point_stroke = [forward_kinematics(starting_angles)]
	coordinates = [starting_point_stroke, *coordinates]
	coordinates = add_stroke_transition_points(coordinates)

	print("\n2D TO 3D - TRANSLATED COORDINATES:")
	for stroke_number, stroke in enumerate(coordinates, start=1):
		for point_number, point in enumerate(stroke, start=1):
			print(f"  stroke {stroke_number}, point {point_number}: {point}")


	angles = coordinates_to_angles(coordinates, starting_angles)
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
		print("Dry run only. Set send_to_robot=True to control Hubert.")



# Quick single-target tests used during setup:
#
# .venv/bin/python main_temp.py --quick-coordinate 0.355 0.15 0.30 --send
# .venv/bin/python main_temp.py --quick-angles 0 -20 40 0 0 0 --send


def main(args):
    image_generator = GoogleImageGenerator()
    audio_listener = AudioListener()
    starting_angles = parse_robot_status(query_robot_status())
    print(f"Initial base, shoulder, and elbow angles: {starting_angles}")
    image_path = get_image_path(
        args,
        image_generator,
        audio_listener,
    )

    draw_image(image_path, args.send, starting_angles=starting_angles)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog="Hubart",
        description="Makes the Hubert robot draw an image."
    )
    parser.add_argument('-d', '--description')
    parser.add_argument('-i', '--image')
    parser.add_argument(
	"--send",
	action="store_true",
	help="send the mapped trajectory to Hubert via serial",
	)
    main(parser.parse_args())