import numpy as np
import pinocchio as pin

# Your physical arm parameters
L1, L4, L7 = 0.055, 0.108, 0.015
L2, L5, L8 = 0.315, 0.005, 0.088
L3, L6, L9 = 0.045, 0.034, 0.204

def rpy_to_matrix(roll, pitch, yaw):
    """Helper to perfectly match URDF/IKPy extrinsic XYZ rotation logic"""
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 1], [0, 0, 1]])
    return Rz @ Ry @ Rx

# --- 1. BUILD THE KINEMATIC TREE ---
model = pin.Model()

# Base Joint (Active) | IKPy bounds: (-pi/2, pi/2)
j1_pose = pin.SE3(rpy_to_matrix(0, 0, 0), np.array([0., 0., 0.055]))
j1_id = model.addJoint(0, pin.JointModelRZ(), j1_pose, "base_joint")

# Shoulder Joint (Active) | IKPy bounds: (-pi/2, pi/3)
j2_pose = pin.SE3(rpy_to_matrix(np.pi/2, -np.pi/2, 0), np.array([L6, -L4, L2 + L3]))
j2_id = model.addJoint(j1_id, pin.JointModelRZ(), j2_pose, "shoulder_joint")

# Elbow Joint (Active) | IKPy bounds: (-pi/2, pi/2)
j3_pose = pin.SE3(rpy_to_matrix(0, 0, 0), np.array([L7, -L8, -L5]))
j3_id = model.addJoint(j2_id, pin.JointModelRZ(), j3_pose, "elbow_joint")

# Gripper (Fixed Operational Frame)
gripper_pose = pin.SE3(rpy_to_matrix(0, 0, 0), np.array([0., -L9-0.12, 0.]))
gripper_frame_id = model.addFrame(
    pin.Frame("gripper", j3_id, 0, gripper_pose, pin.FrameType.OP_FRAME)
)

# Apply exact joint limits matching your previous script
model.lowerPositionLimit = np.array([-np.pi/2, -np.pi/2, -np.pi/2])
model.upperPositionLimit = np.array([ np.pi/2,  np.pi/3,  np.pi/2])

# Allocate memory buffer for C++ calculations (prevents reallocation in the loop)
data = model.createData()

# --- 2. THE HARDCORE C++ SOLVER ---
def get_servo_angles(target_position, previous_angles):
    """
    Solves IK using Damped Least Squares (Levenberg-Marquardt).
    Extremely robust to singularities and executes in microseconds.
    """
    # Start at our previous angles (converted to radians)
    q = np.radians(previous_angles)
    target = np.array(target_position)

    # Solver Parameters
    eps = 1e-6       # Tolerance
    max_iter = 1000  # Cutoff
    damp = 1e-6      # Damping factor to breeze past singularities 
    alpha = 0.5      # Integration step size (keeps the solver stable)

    for _ in range(max_iter):
        # Update forward kinematics and frame placements in C++ memory
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        # Check positional error
        current_pos = data.oMf[gripper_frame_id].translation
        err = current_pos - target

        # If we are within tolerance, we are done
        if np.linalg.norm(err) < eps:
            break

        # Get the Positional Jacobian for the tip (3x3 matrix for 3-DOF arm)
        J = pin.computeFrameJacobian(model, data, q, gripper_frame_id, pin.LOCAL_WORLD_ALIGNED)[:3, :]

        # Damped Least Squares update logic
        # Equivalent to: v = -J^T * (J * J^T + damp * I)^-1 * err
        v = -J.T @ np.linalg.solve(J @ J.T + damp * np.eye(3), err)

        # Integrate velocity to get new position
        q = pin.integrate(model, q, v * alpha)
        
        # Enforce physical hardware bounds inside the loop
        q = np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)

    return tuple(np.degrees(q))