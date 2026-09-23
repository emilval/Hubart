import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

L1, L4, L7 = 0.055, 0.108, 0.015
L2, L5, L8 = 0.315, 0.005, 0.088
L3, L6, L9 = 0.045, 0.034, 0.204

chain = Chain(
    name="Hubart",
    links=[
        OriginLink(),
        URDFLink(
            name="base_joint",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
            bounds=(-np.pi / 2, np.pi / 2),
        ),
        URDFLink(
            name="shoulder_joint",
            origin_translation=[L6, -L4, L2 + L3],
            origin_orientation=[np.pi / 2, -np.pi / 2, 0],
            rotation=[0, 0, 1],
            bounds=(-np.pi / 2, np.pi / 2),
        ),
        URDFLink(
            name="elbow_joint",
            origin_translation=[L7, -L8, -L5],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
            bounds=(-np.pi / 2, np.pi / 2),
        ),
        URDFLink(
            name="gripper_joint",
            origin_translation=[0, -L9, 0],
            origin_orientation=[0, 0, 0],
            rotation=None,
            joint_type="fixed",
        ),
    ],
    active_links_mask=[False, True, True, True, False],
)


def get_servo_angles(target_position):
    """Calculates IK solution and maps joint angles to 0-180° servo degrees."""
    raw_ik_angles = chain.inverse_kinematics(target_position)
    raw_degrees = np.degrees(raw_ik_angles[1:4])  # [base, shoulder, elbow]

    # Map relative angles centered around 90° home position
    base_angle = int(round(raw_degrees[0]))
    shoulder_angle = int(round(raw_degrees[1]))
    elbow_angle = int(round(raw_degrees[2]))

    return base_angle, shoulder_angle, elbow_angle