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
def create_robot_paths(img_path: Path) -> list:
    """
    Extracts single-stroke paths by walking directly along the skeleton pixels.
    Bypasses cv2.findContours to avoid tracing the outside of lines.
    """
    img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    _, binary = cv2.threshold(img, 127, 1, cv2.THRESH_BINARY_INV)
    
    # Create the 1-pixel wide skeleton
    skeleton = skeletonize(binary)
    skeleton_img = (skeleton * 255).astype(np.uint8)
    
    h, w = skeleton_img.shape
    visited = np.zeros_like(skeleton_img, dtype=bool)
    
    def get_neighbors(x, y):
        """Find adjacent unvisited pixels (8-way connected)."""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0: 
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if skeleton_img[ny, nx] > 0 and not visited[ny, nx]:
                        neighbors.append((nx, ny))
        return neighbors

    raw_paths = []
    
    # Scan the image for any unvisited skeleton pixels
    for y in range(h):
        for x in range(w):
            if skeleton_img[y, x] > 0 and not visited[y, x]:
                
                path = [(x, y)]
                visited[y, x] = True
                
                # Walk forward as far as possible
                curr_x, curr_y = x, y
                while True:
                    neighbors = get_neighbors(curr_x, curr_y)
                    if not neighbors:
                        break
                    # Greedily pick the first available path
                    nx, ny = neighbors[0]
                    path.append((nx, ny))
                    visited[ny, nx] = True
                    curr_x, curr_y = nx, ny
                    
                # Because we may have started in the middle of a line, 
                # we must also walk backwards from the initial starting point.
                curr_x, curr_y = x, y
                path_backward = []
                while True:
                    neighbors = get_neighbors(curr_x, curr_y)
                    if not neighbors:
                        break
                    nx, ny = neighbors[0]
                    path_backward.append((nx, ny))
                    visited[ny, nx] = True
                    curr_x, curr_y = nx, ny
                    
                # Stitch the backward and forward paths together
                full_path = path_backward[::-1] + path
                
                if len(full_path) > 2:
                    raw_paths.append(np.array(full_path, dtype=np.int32))

    robot_paths = []
    for path in raw_paths:
        # Simplify the raw pixel path into geometric lines/corners
        # Increase epsilon (e.g., 3.0 or 4.0) for fewer points, decrease for curves
        epsilon = 2.0 
        approx = cv2.approxPolyDP(path, epsilon, closed=False)
        corners = approx.reshape(-1, 2)
        
        # Apply your original filters and interpolations to the clean corners
        if len(corners) >= 2:
            clean_corners = filter_close_points(corners, min_dist=5.0)
            if len(clean_corners) >= 2:
                dense_path = interpolate_path(clean_corners, step_size=5.0)
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
        interval=100, 
        blit=True, 
        repeat=False
    )
    print("Done!")
    plt.show()



def sort_paths_optimized(robot_paths: list) -> list:
    """
    Sorts a list of paths to minimize travel distance.
    Checks both the start and end points of unvisited paths, 
    reversing the stroke direction if the end point is closer.
    """
    if not robot_paths:
        return []

    # Start with the first path in the list
    unvisited = robot_paths[1:]
    sorted_paths = [robot_paths[0]]
    
    # The current position is the end of the very first stroke
    current_pos = sorted_paths[-1][-1]

    while unvisited:
        best_idx = 0
        min_dist = float('inf')
        reverse_path = False
        
        # Check all unvisited paths to find the closest start OR end point
        for i, path in enumerate(unvisited):
            start_point = path[0]
            end_point = path[-1]
            
            # Distance to the start of the path
            dist_to_start = np.linalg.norm(start_point - current_pos)
            # Distance to the end of the path
            dist_to_end = np.linalg.norm(end_point - current_pos)
            
            # Check if start is closest
            if dist_to_start < min_dist:
                min_dist = dist_to_start
                best_idx = i
                reverse_path = False
                
            # Check if end is closest
            if dist_to_end < min_dist:
                min_dist = dist_to_end
                best_idx = i
                reverse_path = True
                
        # Pop the best path from unvisited
        next_path = unvisited.pop(best_idx)
        
        # If the end point was closer, flip the path array so we draw it backwards
        if reverse_path:
            next_path = next_path[::-1]
            
        sorted_paths.append(next_path)
        
        # Update our current position to the new end of this stroke
        current_pos = next_path[-1]

    return sorted_paths


def remove_retraced_segments(robot_paths: list, step_size: float = 5.0) -> list:
    """
    Prevents the robot from drawing over the exact same lines twice (retracing).
    Allows lines to cross (intersections) or share starting points without breaking.
    """
    cleaned_paths = []
    drawn_pixels = set()
    
    # How many consecutive overlapping points count as a "retrace" vs a "crossing".
    # 4 points * 5 step_size = 20 pixels of allowed overlap for intersections.
    intersection_tolerance = 4 
    
    # We mark a 5x5 pixel area around each drawn point.
    # Because step_size is 5.0, this creates a solid "drawn mask" in our memory.
    radius = 2 

    def mark_drawn(point):
        x, y = int(point[0]), int(point[1])
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                drawn_pixels.add((x + dx, y + dy))

    for path in robot_paths:
        if len(path) == 0:
            continue
            
        # Step 1: Identify which points in this stroke overlap with already drawn areas
        overlaps = []
        for pt in path:
            x, y = int(pt[0]), int(pt[1])
            overlaps.append((x, y) in drawn_pixels)
            
        # Step 2: Smooth out small overlaps so intersections don't break the continuous line
        count = 0
        for i in range(len(overlaps)):
            if overlaps[i]:
                count += 1
            else:
                if 0 < count <= intersection_tolerance:
                    # It was a short overlap (a crossing or touch). Revert them to False.
                    for j in range(i - count, i):
                        overlaps[j] = False
                count = 0
                
        # Handle the edge case where a line ends exactly on an intersection
        if 0 < count <= intersection_tolerance:
            for j in range(len(overlaps) - count, len(overlaps)):
                overlaps[j] = False

        # Step 3: Split the path, skipping the segments that are already drawn
        current_subpath = []
        for pt, is_overlap in zip(path, overlaps):
            if not is_overlap:
                current_subpath.append(pt)
            else:
                # We hit a drawn area. Save the current stroke if it has at least 2 points.
                if len(current_subpath) >= 2:
                    cleaned_paths.append(np.array(current_subpath, dtype=np.int32))
                    # Mark this new segment as drawn
                    for p in current_subpath:
                        mark_drawn(p)
                current_subpath = [] # Reset for the next segment
                
        # Add the final stroke segment if it exists
        if len(current_subpath) >= 2:
            cleaned_paths.append(np.array(current_subpath, dtype=np.int32))
            for p in current_subpath:
                mark_drawn(p)
                
    return cleaned_paths

if __name__ == '__main__': 
    image_path = input("Enter image path:")
    robot_paths = create_robot_paths(image_path)
    print(f"Original paths: {len(robot_paths)}")

    robot_paths = remove_retraced_segments(robot_paths)
    print(f"Cleaned down to {len(robot_paths)} unique paths.")

    robot_paths = sort_paths_optimized(robot_paths)
    #scatter_plot_robot_paths(robot_paths)
    plot_robot_paths_animated(robot_paths)