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
            origin_translation=[0, 0, 0.055],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
            bounds=(-np.pi / 2, np.pi / 2),
        ),
        URDFLink(
            name="shoulder_joint",
            origin_translation=[L6, -L4, L2 + L3],
            origin_orientation=[np.pi/2, -np.pi/2, 0],
            rotation=[0, 0, 1],
            bounds=(-np.pi / 2, np.pi / 3),
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
            origin_translation=[0, -L9-0.12, 0],
            origin_orientation=[0, 0, 0],
            rotation=None,
            joint_type="fixed",
        ),
    ],
    active_links_mask=[False, True, True, True, False],
)

strict_options = {
    "max_iter": 1000,
    "ftol": 1e-9,
    "eps": 1e-6
}

def get_servo_angles(target_position, previous_angles):
    """Calculate IK solution and return the 3 joint angles in degrees."""

    # IKPy needs one starting angle for every link.
    starting_angles = np.zeros(len(chain.links))

    # previous_angles are base/shoulder/elbow in degrees.
    # IKPy uses radians.
    starting_angles[1:4] = np.radians(previous_angles)

    raw_ik_angles = chain.inverse_kinematics(
        target_position,
        initial_position=starting_angles, 
        optimizer_budget=1000,
        tol=1e-6,
    )

    # Extract base, shoulder and elbow.
    raw_degrees = np.degrees(raw_ik_angles[1:4])

    return tuple(raw_degrees)
