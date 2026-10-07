import numpy as np
import serial
import time
from svgpathtools import svg2paths2
from control.inverse_kinematics import get_servo_angles

# ==========================================
# 1. Shared inverse kinematics via get_servo_angles
# ==========================================
def calculate_ik(X_P, Y_P, Z_P):
    """
    Batch-compute IK for a sequence of Cartesian points using the shared
    servo-angle solver in control.inverse_kinematics.
    Returns three NumPy arrays of base, shoulder and elbow angles in degrees.
    """
    X_P = np.asarray(X_P, dtype=float)
    Y_P = np.asarray(Y_P, dtype=float)
    Z_P = np.asarray(Z_P, dtype=float)

    theta1, theta2, theta3 = [], [], []
    previous_angles = (0.0, 0.0, 0.0)

    for x, y, z in zip(X_P, Y_P, Z_P):
        servo_angles = get_servo_angles([float(x), float(y), float(z)], previous_angles)
        theta1.append(servo_angles[0])
        theta2.append(servo_angles[1])
        theta3.append(servo_angles[2])
        previous_angles = servo_angles

    return np.asarray(theta1), np.asarray(theta2), np.asarray(theta3)

# ==========================================
# 2. Image parsing function (modified from your script)
# ==========================================
def parse_svg_to_waypoints(svg_file, samples_per_path=50):
    """
    Force the SVG into dense discrete coordinate points.
    Returns a list containing (X_list, Y_list) for each continuous path.
    """
    paths, _, svg_attrs = svg2paths2(svg_file)

    viewbox = svg_attrs.get("viewBox", "0 0 200 200")
    vb_values = list(map(float, viewbox.strip().split()))
    svg_height = vb_values[3] if len(vb_values) == 4 else 200

    all_paths_coords = []

    for path in paths:
        if len(path) == 0:
            continue

        path_x = []
        path_y = []

        for seg in path:
            for t in np.linspace(0, 1, samples_per_path):
                pt = seg.point(t)
                path_x.append(pt.real)
                path_y.append(svg_height - pt.imag)

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
    WALL_X = 0.28

    robot_y_min, robot_y_max = -0.10, 0.10
    robot_z_min, robot_z_max = 0.25, 0.45

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

    for path_idx, (path_y, path_z) in enumerate(mapped_paths):
        print(f"--> Drawing path {path_idx + 1}/{len(mapped_paths)}...")

        X_path = np.full_like(path_y, WALL_X)
        t1, t2, t3 = calculate_ik(X_path, path_y, path_z)

        for i in range(len(path_y)):
            x = float(X_path[i])
            y = float(path_y[i])
            z = float(path_z[i])
            a1 = float(t1[i])
            a2 = float(t2[i])
            a3 = float(t3[i])

            print(
                f"point {i:03d}: coord=(x={x:.4f}, y={y:.4f}, z={z:.4f}) "
                f"angles=(a1={a1:.2f}, a2={a2:.2f}, a3={a3:.2f})"
            )

            command = f"<{a1:.2f},{a2:.2f},{a3:.2f}>\n"
            ser.write(command.encode('utf-8'))

            while True:
                response = ser.readline().decode('utf-8').strip()
                if response == "READY":
                    break

    print("✅ All drawing tasks completed!")
    ser.close()


if __name__ == "__main__":
    draw_svg_with_robot("new_test/square.svg", "/dev/ttyACM0")