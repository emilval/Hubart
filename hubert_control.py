import time
import serial
from inverse_kinematics import get_servo_angles

# =========================================================================
# 1. SET YOUR TARGET COORDINATES HERE (X, Y, Z in meters)
# =========================================================================
TARGET_POSITION = [0.23, 0.13, 0.25]

# Serial Port Settings
PORT = "/dev/tty.usbmodem206EF1333F082"
BAUD_RATE = 115200


def send_target_to_robot(target_xyz):
    # Solve IK via imported module
    base, shoulder, elbow = get_servo_angles(target_xyz)

    print(f"Target position [X, Y, Z]: {target_xyz}")
    print(
        f"Calculated Servo Angles  -> Base: {base}°, Shoulder: {shoulder}°, Elbow: {elbow}°"
    )

    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=2)
        time.sleep(2)  # Wait for ESP32 reset upon connection

        command = f"{base},{shoulder},{elbow}\n"
        print(f"Sending command string: {command.strip()}")
        ser.write(command.encode("utf-8"))

        time.sleep(0.1)
        if ser.in_waiting > 0:
            response = ser.readline().decode("utf-8", errors="ignore").strip()
            print(f"ESP32 Response: {response}")

        ser.close()
        print("Transmission complete.")

    except Exception as e:
        print(f"Serial Error: {e}")


if __name__ == "__main__":
    send_target_to_robot(TARGET_POSITION)