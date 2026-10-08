import argparse
from preprocessing.image_generators import GoogleImageGenerator
from preprocessing.image_pathing import create_robot_paths
from preprocessing.audio_capture import AudioListener
import numpy as np
from pathlib import Path
import re

GEN_IMAGE_PATH = Path("./images/generated_image_test.png")

PROMPT_INJECTION = " It should simple black and white line art with simple lines"

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

def make_hubert_draw(robot_paths: np.array):
    pass

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

    return GEN_IMAGE_PATH

def get_audio_description(audio_listener) -> str:
    while True:
        print("Please state your prompt:")
        description = audio_listener.listen_for_audio()

        feedback = input(
            f'Is this description correct "{description}"? (y/n): '
        )

        if feedback.lower() == "y":
            return description

def main(args):
    image_generator = GoogleImageGenerator()
    audio_listener = AudioListener()

    image_path = get_image_path(
        args,
        image_generator,
        audio_listener,
    )

    robot_paths = create_robot_paths(image_path)
    make_hubert_draw(robot_paths)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog="Hubart",
        description="Makes the Hubert robot draw an image."
    )
    parser.add_argument('-d', '--description')
    parser.add_argument('-i', '--image')

    main(parser.parse_args())