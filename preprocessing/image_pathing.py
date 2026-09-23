import cv2
import numpy as np
from skimage.morphology import skeletonize
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path

def interpolate_path(points, step_size=5.0):
    """
    Fills in the gaps between corner points with evenly spaced points.
    Decrease step_size for denser points, increase for sparser points.
    """
    dense_points = []
    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i + 1]
        
        # Calculate distance between the two points
        dist = np.linalg.norm(p2 - p1)
        
        # Calculate how many points we need based on the step size
        num_steps = max(int(dist / step_size), 1)
        
        # Generate the interpolated X and Y coordinates
        x_vals = np.linspace(p1[0], p2[0], num_steps, endpoint=False)
        y_vals = np.linspace(p1[1], p2[1], num_steps, endpoint=False)
        
        for x, y in zip(x_vals, y_vals):
            dense_points.append([x, y])
            
    # Always ensure the very last point of the stroke is included
    if len(points) > 0:
        dense_points.append(points[-1])
        
    return np.array(dense_points, dtype=np.int32)

def filter_close_points(points, min_dist=3.0):
    """
    Removes points that are too close to the previously accepted point.
    Prevents the robot from stuttering in one spot.
    """
    if len(points) <= 1:
        return points
        
    filtered = [points[0]]
    for pt in points[1:]:
        # Calculate distance from the last accepted point
        dist = np.linalg.norm(pt - filtered[-1])
        
        if dist >= min_dist:
            filtered.append(pt)
            
    # Guarantee the stroke always reaches its exact final destination
    if not np.array_equal(filtered[-1], points[-1]):
        filtered.append(points[-1])

    if np.linalg.norm(filtered[-1] - filtered[0]) < 8.0:
        filtered[-1] = filtered[0]
        
    return np.array(filtered, dtype=np.int32)

def is_duplicate_path(new_path, existing_paths, threshold=5.0):
    """
    Checks if a new path has the exact same bounding box as an existing path.
    This prevents drawing the 'inside' and 'outside' of a 1-pixel skeleton line.
    """
    if len(new_path) == 0:
        return True
        
    min1, max1 = new_path.min(axis=0), new_path.max(axis=0)
    
    for existing_path in existing_paths:
        min2, max2 = existing_path.min(axis=0), existing_path.max(axis=0)
        
        # If the top-left and bottom-right corners are within a few pixels, it's a duplicate
        if np.linalg.norm(min1 - min2) < threshold and np.linalg.norm(max1 - max2) < threshold:
            return True
            
    return False

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
        epsilon = min(max(0.002 * cv2.arcLength(contour, closed=False), 1.0), 3.0)
        approx = cv2.approxPolyDP(contour, epsilon, closed=False)
        corners = approx.reshape(-1, 2)
        if len(corners) > 2:
            clean_corners = filter_close_points(corners, min_dist=5.0)
            
            if len(clean_corners) >= 2:
                dense_path = interpolate_path(clean_corners, step_size=10.0)

                if not is_duplicate_path(dense_path, robot_paths):
                    robot_paths.append(dense_path)
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


def scatter_plot_robot_paths(robot_paths: list) -> None:
    plt.figure(figsize=(8, 8))
    for path in robot_paths:
        x_coords = path[:, 0]
        y_coords = path[:, 1]
        
        # s parameter controls the size of the scatter points
        plt.scatter(x_coords, y_coords, color='black', s=2)

    # Invert Y-axis to match image coordinates
    plt.gca().invert_yaxis()
    plt.axis('equal')
    plt.title("Robot Path Preview - Scatter Plot")
    plt.show()

def plot_robot_paths_animated(robot_paths: list) -> None:
    if not robot_paths:
        print("No paths to animate.")
        return

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("Robot Path Preview - Animated")

    # To keep the camera/view static during animation, we need to pre-calculate 
    # the maximum bounds of all the paths combined and set the axis limits.
    all_x = np.concatenate([path[:, 0] for path in robot_paths])
    all_y = np.concatenate([path[:, 1] for path in robot_paths])
    
    # Adding a small margin of 10 pixels around the drawing
    ax.set_xlim(all_x.min() - 10, all_x.max() + 10)
    ax.set_ylim(all_y.min() - 10, all_y.max() + 10)
    
    ax.invert_yaxis() # Invert Y so it isn't upside down
    ax.set_aspect('equal')

    # Create empty line objects for each path
    lines = [ax.plot([], [], color='black', linewidth=1)[0] for _ in range(len(robot_paths))]

    # Build a list of frames. Each frame tells the animation which stroke we are on 
    # and how many points of that stroke to draw.
    frames = []
    for path_idx, path in enumerate(robot_paths):
        for point_idx in range(1, len(path) + 1):
            frames.append((path_idx, point_idx))

    def update(frame):
        path_idx, point_idx = frame
        
        # Get the X and Y coordinates up to the current point index
        x_coords = robot_paths[path_idx][:point_idx, 0]
        y_coords = robot_paths[path_idx][:point_idx, 1]
        
        # Update the specific line object for this stroke
        lines[path_idx].set_data(x_coords, y_coords)
        return lines

    # Create the animation
    # interval: Delay between frames in milliseconds (lower is faster).
    # blit=True: Optimizes drawing by only re-drawing parts that have changed.
    ani = animation.FuncAnimation(
        fig, 
        update, 
        frames=frames, 
        interval=200, 
        blit=True, 
        repeat=False
    )

    plt.show()

if __name__ == '__main__': 
    image_path = input("Enter image path:")
    robot_paths = create_robot_paths(image_path)
    print(robot_paths)
    #scatter_plot_robot_paths(robot_paths)
    plot_robot_paths_animated(robot_paths)