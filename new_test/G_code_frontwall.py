import numpy as np
import serial
import time
from svgpathtools import svg2paths2

# ==========================================
# 1. Pure mathematical inverse kinematics
# ==========================================
def calculate_ik(X_P, Y_P, Z_P):
    L2, L3, L4, L5 = 0.315, 0.045, 0.108, 0.005
    L6, L7, L8, L9 = 0.034, 0.015, 0.088, 0.204
    
    X_P, Y_P, Z_P = np.asarray(X_P), np.asarray(Y_P), np.asarray(Z_P)
    
    r_sq = X_P**2 + Y_P**2
    offset = L4 - L5
    theta1 = np.arctan2(Y_P, X_P) + np.arctan2(offset, np.sqrt(np.maximum(0, r_sq - offset**2)))
    
    X1 = X_P * np.cos(theta1) + Y_P * np.sin(theta1) - L6
    Y1 = Z_P - L2 - L3
    
    E = -2 * (X1 * L7 - Y1 * L8)
    F = -2 * (X1 * L8 + Y1 * L7)
    G = L9**2 - X1**2 - Y1**2 - L7**2 - L8**2
    inner_sqrt = E**2 + F**2 - G**2
    theta2 = np.arctan2(G, np.sqrt(np.maximum(0, inner_sqrt))) - np.arctan2(E, F)
    
    M = X1 - L7 * np.cos(theta2) - L8 * np.sin(theta2)
    N = Y1 - L7 * np.sin(theta2) + L8 * np.cos(theta2)
    theta3 = np.arctan2(M, -N) - theta2
    
    return np.degrees(theta1), np.degrees(theta2), np.degrees(theta3)

# ==========================================
# 2. SVG forced-closed parsing
# ==========================================
def parse_svg_to_waypoints(svg_file, samples_per_path=50):
    paths, attributes, svg_attrs = svg2paths2(svg_file)
    viewbox = svg_attrs.get("viewBox", "0 0 200 200")
    vb_values = list(map(float, viewbox.strip().split()))
    svg_height = vb_values[3] if len(vb_values) == 4 else 200

    all_paths_coords = []
    for path in paths:
        if len(path) == 0: continue
        path_x, path_y = [], []
        
        for seg in path:
            for t in np.linspace(0, 1, samples_per_path):
                pt = seg.point(t)
                path_x.append(pt.real)
                path_y.append(svg_height - pt.imag)
                
        # Core fix: check whether the start and end points coincide. If not, forcibly generate a final closing straight line to seal the groove.
        if len(path_x) > 0:
            dist = (path_x[0] - path_x[-1])**2 + (path_y[0] - path_y[-1])**2
            if dist > 1e-4:
                for t in np.linspace(0, 1, samples_per_path):
                    curr_x = path_x[-1] + t * (path_x[0] - path_x[-1])
                    curr_y = path_y[-1] + t * (path_y[0] - path_y[-1])
                    path_x.append(curr_x)
                    path_y.append(curr_y)
                    
        all_paths_coords.append((np.array(path_x), np.array(path_y)))
    return all_paths_coords

# ==========================================
# 3. Hardware drawing with pen-lift mechanism
# ==========================================
def draw_svg_with_robot(svg_file, port, baudrate=115200):
    print("1. Discretizing SVG paths...")
    raw_paths = parse_svg_to_waypoints(svg_file)
    
    all_x = np.concatenate([p[0] for p in raw_paths])
    all_y = np.concatenate([p[1] for p in raw_paths])
    min_x, max_x = np.min(all_x), np.max(all_x)
    min_y, max_y = np.min(all_y), np.max(all_y)
    
    print("2. Mapping image coordinates to a VERTICAL WALL (YZ Plane)...")
    WALL_X = 0.28   # Drawing board depth
    HOVER_X = 0.23  # Pen-lift safe depth (retract 5 cm)
    
    robot_y_min, robot_y_max = -0.05, 0.05  
    robot_z_min, robot_z_max = 0.20, 0.30   
    
    mapped_paths = []
    for (rx, ry) in raw_paths:
        my = np.interp(rx, [min_x, max_x], [robot_y_min, robot_y_max])
        mz = np.interp(ry, [min_y, max_y], [robot_z_min, robot_z_max])
        mapped_paths.append((my, mz))

    print(f"3. Attempting to connect to robotic arm ({port})...")
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
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
    
    plunge_steps = 10  # Split the 5 cm pen-up/pen-down motion into 10 steps, forcing a straight line along the X-axis

    for path_idx, (path_y, path_z) in enumerate(mapped_paths):
        print(f"--> Drawing path {path_idx + 1}/{len(mapped_paths)}...")
        
        # A. Move above the start point (pen lifted; first ensure it is positioned at a safe height)
        t1_h, t2_h, t3_h = calculate_ik(np.array([HOVER_X]), np.array([path_y[0]]), np.array([path_z[0]]))
        ser.write(f"<{t1_h[0]:.2f},{t2_h[0]:.2f},{t3_h[0]:.2f}>\n".encode('utf-8'))
        while True:
            if ser.readline().decode('utf-8').strip() == "READY": break
            
        # B. Plunge in a straight line (slowly poke toward the drawing board along the X-axis)
        for step in range(1, plunge_steps + 1):
            curr_x = HOVER_X + (WALL_X - HOVER_X) * (step / plunge_steps)
            t1, t2, t3 = calculate_ik(np.array([curr_x]), np.array([path_y[0]]), np.array([path_z[0]]))
            ser.write(f"<{t1[0]:.2f},{t2[0]:.2f},{t3[0]:.2f}>\n".encode('utf-8'))
            while True:
                if ser.readline().decode('utf-8').strip() == "READY": break

        # C. Draw the image contour against the wall
        X_path = np.full_like(path_y, WALL_X)
        t1, t2, t3 = calculate_ik(X_path, path_y, path_z)
        
        for i in range(len(path_y)):
            command = f"<{t1[i]:.2f},{t2[i]:.2f},{t3[i]:.2f}>\n"
            ser.write(command.encode('utf-8'))
            while True:
                if ser.readline().decode('utf-8').strip() == "READY": break

        # D. Lift the pen in a straight line (after drawing, pull out perpendicularly along the X-axis)
        for step in range(1, plunge_steps + 1):
            curr_x = WALL_X - (WALL_X - HOVER_X) * (step / plunge_steps)
            t1, t2, t3 = calculate_ik(np.array([curr_x]), np.array([path_y[-1]]), np.array([path_z[-1]]))
            ser.write(f"<{t1[0]:.2f},{t2[0]:.2f},{t3[0]:.2f}>\n".encode('utf-8'))
            while True:
                if ser.readline().decode('utf-8').strip() == "READY": break

    print("✅ All drawing tasks completed!")
    ser.close()

if __name__ == "__main__":
    # Please make sure the port number here exactly matches the one in your Arduino IDE
    draw_svg_with_robot("square.svg", "COM4")