import numpy as np
import serial
import time
from svgpathtools import svg2paths2, Path

# ==========================================
# 1. Previously written inverse kinematics calculation function
# ==========================================
def calculate_ik(X_P, Y_P, Z_P):
    """
    Batch-compute the inverse kinematics for the Hubert robot.
    X_P, Y_P, Z_P should be passed as coordinate arrays in meters.
    """
    # Robot arm parameters (unit: m)
    L2, L3, L4, L5 = 0.315, 0.045, 0.108, 0.005
    L6, L7, L8, L9 = 0.034, 0.015, 0.088, 0.204
    
    X_P, Y_P, Z_P = np.asarray(X_P), np.asarray(Y_P), np.asarray(Z_P)
    
    # Solve Theta 1
    r_sq = X_P**2 + Y_P**2
    offset = L4 - L5
    theta1 = np.arctan2(Y_P, X_P) + np.arctan2(offset, np.sqrt(np.maximum(0, r_sq - offset**2)))
    
    # Solve intermediate variables
    X1 = X_P * np.cos(theta1) + Y_P * np.sin(theta1) - L6
    Y1 = Z_P - L2 - L3
    
    # Solve Theta 2
    E = -2 * (X1 * L7 - Y1 * L8)
    F = -2 * (X1 * L8 + Y1 * L7)
    G = L9**2 - X1**2 - Y1**2 - L7**2 - L8**2
    inner_sqrt = E**2 + F**2 - G**2
    theta2 = np.arctan2(G, np.sqrt(np.maximum(0, inner_sqrt))) - np.arctan2(E, F)
    
    # Solve Theta 3
    M = X1 - L7 * np.cos(theta2) - L8 * np.sin(theta2)
    N = Y1 - L7 * np.sin(theta2) + L8 * np.cos(theta2)
    theta3 = np.arctan2(M, -N) - theta2
    
    return np.degrees(theta1), np.degrees(theta2), np.degrees(theta3)

# ==========================================
# 2. Image parsing function (modified from your script)
# ==========================================
def parse_svg_to_waypoints(svg_file, samples_per_path=50):
    """
    Force the SVG into dense discrete coordinate points.
    Returns a list containing (X_list, Y_list) for each continuous path.
    """
    paths, attributes, svg_attrs = svg2paths2(svg_file)
    
    viewbox = svg_attrs.get("viewBox", "0 0 200 200")
    vb_values = list(map(float, viewbox.strip().split()))
    svg_height = vb_values[3] if len(vb_values) == 4 else 200

    all_paths_coords = []
    
    for path in paths:
        if len(path) == 0: continue
        
        path_x = []
        path_y = []
        
        # Discretize all segments, regardless of shape, using .point(t)
        for seg in path:
            # Dynamically adjusting the number of sample points based on segment length can improve accuracy; a fixed value is used here for simplicity.
            for t in np.linspace(0, 1, samples_per_path):
                pt = seg.point(t)
                path_x.append(pt.real)
                path_y.append(svg_height - pt.imag) # Flip the Y-axis
                
        all_paths_coords.append((np.array(path_x), np.array(path_y)))
        
    return all_paths_coords

# ==========================================
# 3. Mapping and sending function
# ==========================================
def draw_svg_with_robot(svg_file, port, baudrate=115200):
    
    print("1. Discretizing SVG paths...")
    raw_paths = parse_svg_to_waypoints(svg_file)
    
    all_x = np.concatenate([p[0] for p in raw_paths])
    all_y = np.concatenate([p[1] for p in raw_paths])
    min_x, max_x = np.min(all_x), np.max(all_x)
    min_y, max_y = np.min(all_y), np.max(all_y)
    
    print("2. Mapping image coordinates to a VERTICAL WALL (YZ Plane)...")
    # define the fixed depth (X-axis) for the robotic arm to the wall (unit: meters). Adjust this value based on your physical drawing board's distance.
    WALL_X = 0.28 
    
    robot_y_min, robot_y_max = -0.10, 0.10  # the length between -0.10 and 0.10 meters
    robot_z_min, robot_z_max = 0.25, 0.45   # the height between 0.25 and 0.45 meters 
    
    mapped_paths = []
    for (rx, ry) in raw_paths:
        # map X (width) to the robot's Y (left-right)
        my = np.interp(rx, [min_x, max_x], [robot_y_min, robot_y_max])
        # map the image's Y (height) to the robot's Z (up-down)
        mz = np.interp(ry, [min_y, max_y], [robot_z_min, robot_z_max])
        mapped_paths.append((my, mz))

    print(f"3. Attempting to connect to robotic arm ({port})...")
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        # force DTR/RTS signals to reset the ESP32 hardware, avoiding lockup
        ser.setDTR(False)
        ser.setRTS(False)
        time.sleep(0.1)
        ser.setDTR(True)
        ser.setRTS(True)
        time.sleep(2) 
        ser.reset_input_buffer()
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    print("Robotic arm is ready, starting to send trajectory!")
    
    for path_idx, (path_y, path_z) in enumerate(mapped_paths):
        print(f"--> Drawing path {path_idx + 1}/{len(mapped_paths)}...")
        
        # X axis (depth) becomes a fixed array
        X_path = np.full_like(path_y, WALL_X)
        
        # Pass in X(fixed), Y(variable), Z(variable) for inverse kinematics solving
        t1, t2, t3 = calculate_ik(X_path, path_y, path_z)
        
        for i in range(len(path_y)):
            command = f"<{t1[i]:.2f},{t2[i]:.2f},{t3[i]:.2f}>\n"
            ser.write(command.encode('utf-8'))
            
            while True:
                response = ser.readline().decode('utf-8').strip()
                if response == "READY":
                    break

    print("✅ All drawing tasks completed!")
    ser.close()

if __name__ == "__main__":
    draw_svg_with_robot("square.svg", "COM4")