import argparse
from preprocessing.image_generators import GoogleImageGenerator
from preprocessing.image_pathing import create_robot_paths
import numpy as np
from pathlib import Path

GEN_IMAGE_PATH = Path("./images/generated_image.png")

def make_hubert_draw(robot_paths: np.array):
    pass

def main(args):

    image_generator = GoogleImageGenerator()

    # Choose how to get the image to draw
    if args.image:
        image_path = Path(args.image)

    elif args.description:
        image_description = args.description
        image_generator.generate_image(prompt=image_description, output_path=GEN_IMAGE_PATH)
        image_path = GEN_IMAGE_PATH

    else:
        pass

    robot_paths = create_robot_paths(image_path)

    make_hubert_draw(robot_paths=robot_paths)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog="Hubart",
        description="Makes the Hubert robot draw an image."
    )
    parser.add_argument('-d', '--description')
    parser.add_argument('-i', '--image')

    main(parser.parse_args())