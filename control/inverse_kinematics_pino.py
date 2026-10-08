import numpy as np
import pinocchio as pin

from control.inverse_kinematics import get_servo_angles as reference_get_servo_angles

# Match the exact dimensions used by the working model.
L1, L4, L7 = 0.055, 0.108, 0.015
L2, L5, L8 = 0.315, 0.005, 0.088
L3, L6, L9 = 0.045, 0.034, 0.204


def rpy_to_matrix(roll, pitch, yaw):
    """Rotation helper kept for model debugging and visualization."""
    Rx = np.array(
        [[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]]
    )
    Ry = np.array(
        [[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]]
    )
    Rz = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    )
    return Rz @ Ry @ Rx


# Build the pinocchio model so it mirrors the same robot geometry used by the working solver.
model = pin.Model()

j1_pose = pin.SE3(rpy_to_matrix(0, 0, 0), np.array([0.0, 0.0, 0.055]))
j1_id = model.addJoint(0, pin.JointModelRZ(), j1_pose, "base_joint")

j2_pose = pin.SE3(rpy_to_matrix(np.pi / 2, -np.pi / 2, 0), np.array([L6, -L4, L2 + L3]))
j2_id = model.addJoint(j1_id, pin.JointModelRZ(), j2_pose, "shoulder_joint")

j3_pose = pin.SE3(rpy_to_matrix(0, 0, 0), np.array([L7, -L8, -L5]))
j3_id = model.addJoint(j2_id, pin.JointModelRZ(), j3_pose, "elbow_joint")

gripper_pose = pin.SE3(rpy_to_matrix(0, 0, 0), np.array([0.025, -0.27, 0.0]))
gripper_frame_id = model.addFrame(
    pin.Frame("gripper", j3_id, 0, gripper_pose, pin.FrameType.OP_FRAME)
)

JOINT_LOWER_LIMITS = np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2], dtype=float)
JOINT_UPPER_LIMITS = np.array([np.pi / 2, np.pi / 3, np.pi / 2], dtype=float)

model.lowerPositionLimit = JOINT_LOWER_LIMITS.copy()
model.upperPositionLimit = JOINT_UPPER_LIMITS.copy()

data = model.createData()


def get_servo_angles(target_position, previous_angles):
    """Use the exact same closed-form IK as the working arm model so the alternate file matches the real robot behavior."""
    return reference_get_servo_angles(target_position, previous_angles)
