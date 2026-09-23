import cv2
import numpy as np
from skimage.morphology import skeletonize
import matplotlib.pyplot as plt
from pathlib import Path

def create_robot_paths(img_path: Path) -> np.array:
    """
    Produces an array of arrays, where each array is a pen stroke. Epsilon controls
    the amount of substrokes within each stroke.
    """

    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    _, binary = cv2.threshold(img, 127, 1, cv2.THRESH_BINARY_INV)
    skeleton = skeletonize(binary)

    skeleton_img = (skeleton * 255).astype(np.uint8)
    contours, _ = cv2.findContours(skeleton_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    robot_paths = []
    for contour in contours:
        epsilon = 1.0 
        approx = cv2.approxPolyDP(contour, epsilon, closed=False)
        points = approx.reshape(-1, 2)
        if len(points) > 5:
            robot_paths.append(points)
    print(f"Success! Extracted {len(robot_paths)} single-stroke paths for the robot.")
    return robot_paths




def plot_robot_paths(robot_paths: np.array) -> None:
    plt.figure(figsize=(8, 8))
    for path in robot_paths:
        x_coords = path[:, 0]
        y_coords = path[:, 1]
        
        plt.plot(x_coords, y_coords, color='black', linewidth=1)

    # Image coordinates have (0,0) at the top-left, 
    # but graphs normally have (0,0) at the bottom-left. 
    # We must invert the Y-axis so the drawing isn't upside down!
    plt.gca().invert_yaxis()
    plt.axis('equal')
    plt.title("Robot Path Preview")
    plt.show()

if __name__ == '__main__': 
    image_path = input("Enter image path:")
    robot_paths = create_robot_paths(image_path)
    print(robot_paths)
    plot_robot_paths(robot_paths)