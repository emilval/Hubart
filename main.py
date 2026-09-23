import argparse
from preprocessing.image_generators import GoogleImageGenerator
from preprocessing.image_pathing import create_robot_paths
from preprocessing.audio_capture import AudioListener
import numpy as np
from pathlib import Path

GEN_IMAGE_PATH = Path("./images/generated_image.png")

def make_hubert_draw(robot_paths: np.array):
    pass

def get_image_path(args, image_generator, audio_listener) -> Path:
    if args.image:
        return Path(args.image)

    description = args.description

    if not description:
        description = get_audio_description(audio_listener)

    image_generator.generate_image(
        prompt=description,
        output_path=GEN_IMAGE_PATH
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