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
            origin_orientation=[np.pi / 2, -np.pi / 2, 0],
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
            origin_translation=[0, -L9 - 0.12, 0],
            origin_orientation=[0, 0, 0],
            rotation=None,
            joint_type="fixed",
        ),
    ],

    active_links_mask=[False, True, True, True, False],
)


def interpolate_cartesian_path(points, samples_per_segment=20):

    points = np.asarray(points, dtype=float)

    if points.size == 0:
        return np.empty((0, 3), dtype=float)

    if points.ndim == 1:
        points = points.reshape(1, -1)

    if points.shape[1] != 3:
        raise ValueError("Expected (N, 3) Cartesian coordinates.")

    if len(points) < 2:
        return points.copy()

    result = []

    for segment_index, (start, end) in enumerate(
        zip(points[:-1], points[1:])
    ):

        segment = np.linspace(
            start,
            end,
            samples_per_segment + 1
        )

        if segment_index > 0:
            segment = segment[1:]

        result.extend(segment)

    return np.asarray(result)


def get_servo_angles(target_position, previous_angles):

    starting_angles = np.zeros(len(chain.links))

    starting_angles[1:4] = np.radians(previous_angles)

    raw_ik_angles = chain.inverse_kinematics(
        target_position,
        initial_position=starting_angles,
        optimizer_budget=1000,
        tol=1e-6,
    )

    raw_degrees = np.degrees(raw_ik_angles[1:4])

    return tuple(raw_degrees)


def cartesian_path_to_servo_angles(
    points,
    previous_angles=(0.0, 0.0, 0.0),
    samples_per_segment=20
):

    interpolated = interpolate_cartesian_path(
        points,
        samples_per_segment=samples_per_segment
    )

    angle_commands = []

    current_previous = previous_angles

    for point in interpolated:

        servo_angles = get_servo_angles(
            point,
            current_previous
        )

        angle_commands.append(servo_angles)

        current_previous = servo_angles

    return angle_commands
