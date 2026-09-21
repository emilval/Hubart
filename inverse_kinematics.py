import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

L1, L4, L7= 0.055, 0.108, 0.015
L2, L5, L8 = 0.315, 0.005, 0.088
L3, L6, L9 = 0.045, 0.034, 0.204


chain = Chain(name="Hubart", links=[
    OriginLink(),
    URDFLink(
        name="base_joint",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
        bounds=(-np.pi/2, np.pi/2)
    ),
    URDFLink(
        name="shoulder_joint",
        origin_translation=[L6, -L4, L2 + L3],
        origin_orientation=[np.pi/2, -np.pi/2, 0],
        rotation=[0, 0, 1],
        bounds=(-np.pi/2, np.pi/2)
    ),
    URDFLink(
        name="elbow_joint",
        origin_translation=[L7, -L8, -L5],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
        bounds=(-np.pi/2, np.pi/2)
    ),
    URDFLink(
        name="gripper_joint",
        origin_translation=[0, -L9, 0],
        origin_orientation=[0, 0, 0],
        rotation=None,
        joint_type="fixed"
        )
    ],
    active_links_mask=[False, True, True, True, False]
    )
Target_position = [0.22, 0.13, 0.25]
angles = chain.inverse_kinematics(Target_position)
print("Joint angles to reach the target position:", np.degrees(angles[1:]))
