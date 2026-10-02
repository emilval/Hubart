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
    
    # Get the bounding box of the original graphic for coordinate mapping
    all_x = np.concatenate([p[0] for p in raw_paths])
    all_y = np.concatenate([p[1] for p in raw_paths])
    min_x, max_x = np.min(all_x), np.max(all_x)
    min_y, max_y = np.min(all_y), np.max(all_y)
    
    print("2. Mapping image coordinates to the robot workspace (unit: meters)...")
    # Define a safe drawing area in front of the robot arm (e.g., 0.2-0.3 m from the base front edge, 0.2 m wide left to right)
    robot_x_min, robot_x_max = 0.20, 0.30 
    robot_y_min, robot_y_max = -0.10, 0.10
    
    mapped_paths = []
    for (rx, ry) in raw_paths:
        # Linearly map image coordinates to the physical workspace
        mx = np.interp(rx, [min_x, max_x], [robot_x_min, robot_x_max])
        my = np.interp(ry, [min_y, max_y], [robot_y_min, robot_y_max])
        mapped_paths.append((mx, my))

    print(f"3. Attempting to connect to the robot arm ({port})...")
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        ser.setDTR(False)
        ser.setRTS(False)
        time.sleep(0.1)
        ser.setDTR(True)
        ser.setRTS(True)
        
        # give ESP32 two seconds to reset and initialize
        time.sleep(2) 
        ser.reset_input_buffer()
        
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return
            
    print("Robot arm is ready. Starting to send trajectory!")
    
    Z_DRAW = 0.329 # Set pen-down height (m) (adjust according to the previous problem's calculation or measured height)
    
    for path_idx, (path_x, path_y) in enumerate(mapped_paths):
        print(f"--> Drawing path {path_idx + 1}/{len(mapped_paths)}...")
        
        Z_path = np.full_like(path_x, Z_DRAW)
        t1, t2, t3 = calculate_ik(path_x, path_y, Z_path)
        
        for i in range(len(path_x)):
            # Here you can add extra serial commands to control the gripper servo to implement "pen up" and "pen down"
            command = f"<{t1[i]:.2f},{t2[i]:.2f},{t3[i]:.2f}>"
            ser.write(command.encode('utf-8'))
            
            # Block and wait for the microcontroller to respond with "READY"
            while True:
                response = ser.readline().decode('utf-8').strip()
                if response == "READY":
                    break

    print("✅ All drawing tasks completed!")
    ser.close()

if __name__ == "__main__":
    draw_svg_with_robot("square.svg", "COM4")