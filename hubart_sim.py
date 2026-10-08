"""Visualize Hubart's IK model and optionally animate an image path."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider

from control.inverse_kinematics import (
    L1,
    L2,
    L3,
    L4,
    L5,
    L6,
    L7,
    L8,
    L9,
    chain,
    get_servo_angles,
)


# Edit these values to preview a single Cartesian target or an image trajectory.
TARGET_POSITION = (0.446, -0.103, 0.455)  # (x, y, z), meters; zero-joint tool pose
# Defaults to main_temp.py's configured drawing image; set to None for a static target.
IMAGE_PATH: str | Path | None = Path(__file__).resolve().parent / "images" / "square.png"

PIECE_SPECS = (
    ("L1", L1, 0, (0.0, 0.0, 1.0)),
    ("L6", L6, 1, (1.0, 0.0, 0.0)),
    ("L4", L4, 1, (0.0, -1.0, 0.0)),
    ("L2", L2, 1, (0.0, 0.0, 1.0)),
    ("L3", L3, 1, (0.0, 0.0, 1.0)),
    ("L7", L7, 2, (1.0, 0.0, 0.0)),
    ("L8", L8, 2, (0.0, -1.0, 0.0)),
    ("L5", L5, 2, (0.0, 0.0, -1.0)),
    ("Tool mount", 0.025, 3, (1.0, 0.0, 0.0)),
    ("L9", L9, 3, (0.0, -1.0, 0.0)),
    ("Tool extension", 0.12, 3, (0.0, -1.0, 0.0)),
)
PIECE_COLORS = (
    "#34495e",
    "#e67e22",
    "#d35400",
    "#159a9c",
    "#1abc9c",
    "#2980b9",
    "#3498db",
    "#5dade2",
    "#7f8c8d",
    "#c0392b",
    "#e74c3c",
)
VIEW_PRESETS = {
    "Top": (90, -90),
    "Bottom": (-90, -90),
    "Front": (0, 0),
    "Back": (0, 180),
    "Left": (0, 90),
    "Right": (0, -90),
    "Iso": (30, -60),
}


def joint_positions(servo_angles: tuple[float, float, float]) -> np.ndarray:
    """Return the modeled link origins in world coordinates, in meters."""
    angles = np.zeros(len(chain.links), dtype=float)
    angles[1:4] = np.radians(servo_angles)
    transforms = chain.forward_kinematics(angles, full_kinematics=True)
    return np.asarray([transform[:3, 3] for transform in transforms])


def piece_positions(servo_angles: tuple[float, float, float]) -> np.ndarray:
    """Return start/end coordinates for each named length in the IK model."""
    angles = np.zeros(len(chain.links), dtype=float)
    angles[1:4] = np.radians(servo_angles)
    transforms = chain.forward_kinematics(angles, full_kinematics=True)
    pieces = []
    cursors = {index: transforms[index][:3, 3].copy() for index in range(len(transforms))}

    for _name, length, parent_index, local_direction in PIECE_SPECS:
        parent_transform = transforms[parent_index]
        start = cursors[parent_index]
        local_offset = np.asarray(local_direction, dtype=float) * length
        end = start + parent_transform[:3, :3] @ local_offset
        pieces.append((start, end))
        cursors[parent_index] = end

    return np.asarray(pieces)


@dataclass
class ImageTrajectory:
    segments: list[np.ndarray]
    is_drawing: list[bool]
    servo_angles: list[tuple[float, float, float]]


def prepare_image_trajectory(image_path: Path) -> ImageTrajectory:
    """Run the image through the same mapping, transitions, and IK as main_temp.py."""
    from main_temp import (
        COORDINATE_DECIMAL_PLACES,
        add_stroke_transition_points,
        coordinates_to_angles,
        map_image_to_3d,
    )
    from preprocessing.image_pathing import create_robot_paths

    pixel_paths = create_robot_paths(image_path)
    if not pixel_paths:
        raise ValueError(f"No drawable paths found in image: {image_path}")

    mapped_strokes = [
        [
            (
                point[0],
                round(float(point[1]), COORDINATE_DECIMAL_PLACES),
                round(float(point[2]), COORDINATE_DECIMAL_PLACES),
            )
            for point in stroke
        ]
        for stroke in map_image_to_3d(pixel_paths)
    ]
    pipeline_segments = add_stroke_transition_points(mapped_strokes)
    angle_commands = coordinates_to_angles(pipeline_segments)

    segments = [np.asarray(segment, dtype=float) for segment in pipeline_segments]
    is_drawing = [segment_index % 2 == 0 for segment_index in range(len(segments))]
    servo_angles = [tuple(command[:3]) for command in angle_commands]
    if sum(len(segment) for segment in segments) != len(servo_angles):
        raise RuntimeError("main_temp pipeline returned mismatched coordinates and IK commands")

    return ImageTrajectory(segments, is_drawing, servo_angles)


def set_equal_axes(ax, positions: np.ndarray, targets: np.ndarray) -> None:
    points = np.vstack((positions, targets))
    lower = np.min(points, axis=0)
    upper = np.max(points, axis=0)
    center = (lower + upper) / 2
    span = max(float(np.max(upper - lower)), 0.25)
    half_span = span * 0.65
    ax.set_xlim(center[0] - half_span, center[0] + half_span)
    ax.set_ylim(center[1] - half_span, center[1] + half_span)
    ax.set_zlim(max(0.0, center[2] - half_span), center[2] + half_span)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_xticks(np.linspace(*ax.get_xlim(), 5))
    ax.set_yticks(np.linspace(*ax.get_ylim(), 5))
    ax.set_zticks(np.linspace(*ax.get_zlim(), 5))
    ax.grid(True, alpha=0.35)


def draw_simulation(trajectory: ImageTrajectory | None = None) -> None:
    if trajectory:
        targets = np.vstack(trajectory.segments)
        initial_angles = trajectory.servo_angles[0]
        title = "Hubart | Image path simulation"
    else:
        targets = np.asarray([TARGET_POSITION], dtype=float)
        initial_angles = get_servo_angles(TARGET_POSITION, (0.0, 0.0, 0.0))
        frames = []
        title = "Hubart | Cartesian target simulation"

    initial_positions = joint_positions(initial_angles)
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(title)
    set_equal_axes(ax, initial_positions, targets)

    if trajectory:
        for segment, is_drawing in zip(trajectory.segments, trajectory.is_drawing):
            ax.plot(
                segment[:, 0],
                segment[:, 1],
                segment[:, 2],
                color="#b03a2e" if is_drawing else "#7f8c8d",
                linestyle="-" if is_drawing else "--",
                linewidth=1.2,
            )

    ax.scatter(*targets.T, color="#b03a2e", s=18, alpha=0.45, label="Target path")
    robot_lines = [
        ax.plot([], [], [], color=color, linewidth=6, solid_capstyle="round")[0]
        for color in PIECE_COLORS
    ]
    joint_scatter = ax.scatter([], [], [], color="#1b2631", s=35, depthshade=False)
    tool_marker = ax.scatter([], [], [], color="#c0392b", s=55, depthshade=False, label="Tool tip")
    target_marker = ax.scatter([], [], [], color="#f1c40f", s=70, marker="x", label="Current target")
    dimension_labels = [ax.text(0, 0, 0, "") for _ in PIECE_SPECS]
    joint_labels = [ax.text(0, 0, 0, "") for _ in range(len(chain.links))]
    info = ax.text2D(0.02, 0.97, "", transform=ax.transAxes, va="top", family="monospace")
    ax.legend(loc="upper right")

    fig.subplots_adjust(left=0.04, right=0.96, top=0.92, bottom=0.22 if trajectory else 0.14)
    button_left = 0.08
    button_right = 0.92
    button_gap = 0.012
    button_width = (button_right - button_left - button_gap * (len(VIEW_PRESETS) - 1)) / len(VIEW_PRESETS)
    view_buttons = []

    def set_view(elevation: int, azimuth: int) -> None:
        ax.view_init(elev=elevation, azim=azimuth)
        fig.canvas.draw_idle()

    for index, (label, (elevation, azimuth)) in enumerate(VIEW_PRESETS.items()):
        button_ax = fig.add_axes(
            [button_left + index * (button_width + button_gap), 0.035, button_width, 0.055]
        )
        button = Button(button_ax, label, color="#ecf0f1", hovercolor="#d5d8dc")
        button.label.set_fontsize(9)
        button.on_clicked(lambda _event, elev=elevation, azim=azimuth: set_view(elev, azim))
        view_buttons.append(button)

    set_view(*VIEW_PRESETS["Iso"])
    fig._hubart_view_buttons = view_buttons

    trails = [
        ax.plot(
            [], [], [],
            color="#c0392b" if is_drawing else "#7f8c8d",
            linewidth=2,
            linestyle="-" if is_drawing else "--",
        )[0]
        for is_drawing in trajectory.is_drawing
    ] if trajectory else []

    def update_pose(
        servo_angles: tuple[float, float, float],
        target: np.ndarray,
        phase: str = "",
    ) -> None:
        positions = joint_positions(servo_angles)
        pieces = piece_positions(servo_angles)
        for index, line in enumerate(robot_lines):
            piece = pieces[index]
            line.set_data(piece[:, 0], piece[:, 1])
            line.set_3d_properties(piece[:, 2])
        joint_scatter._offsets3d = (positions[:, 0], positions[:, 1], positions[:, 2])
        tool = positions[-1]
        tool_marker._offsets3d = ([tool[0]], [tool[1]], [tool[2]])
        target_marker._offsets3d = ([target[0]], [target[1]], [target[2]])

        for index, (piece, spec) in enumerate(zip(pieces, PIECE_SPECS)):
            start, end = piece
            name, length, _, _ = spec
            midpoint = (start + end) / 2
            dimension_labels[index].set_position((midpoint[0], midpoint[1]))
            dimension_labels[index].set_3d_properties(midpoint[2])
            dimension_labels[index].set_text(f"{name} {length * 1000:.0f} mm")
            dimension_labels[index].set_color(PIECE_COLORS[index])

        for index, (position, name) in enumerate(zip(positions, ("Origin", "Base", "Shoulder", "Elbow", "Tool"))):
            joint_labels[index].set_position((position[0], position[1]))
            joint_labels[index].set_3d_properties(position[2])
            joint_labels[index].set_text(name)

        distance = float(np.linalg.norm(target))
        error = float(np.linalg.norm(target - tool))
        phase_label = f"{phase}\n" if phase else ""
        info.set_text(
            f"{phase_label}Target: ({target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f}) m\n"
            f"Tool:   ({tool[0]:+.3f}, {tool[1]:+.3f}, {tool[2]:+.3f}) m\n"
            f"Origin-to-target: {distance:.3f} m ({distance * 1000:.0f} mm)\n"
            f"Target error: {error * 1000:.1f} mm\n"
            f"Servo angles: {servo_angles[0]:.1f}, {servo_angles[1]:.1f}, {servo_angles[2]:.1f} deg"
        )

    if not trajectory:
        update_pose(initial_angles, targets[0])
    else:
        def update_frame(frame_index: int):
            segment_index = 0
            point_index = frame_index
            for segment_index, segment in enumerate(trajectory.segments):
                if point_index < len(segment):
                    break
                point_index -= len(segment)

            angles = trajectory.servo_angles[frame_index]
            target = trajectory.segments[segment_index][point_index]
            phase = "Drawing" if trajectory.is_drawing[segment_index] else "Lift / transition"
            update_pose(angles, target, phase)
            for trail_index, (path, line) in enumerate(zip(trajectory.segments, trails)):
                if trail_index < segment_index:
                    visible_path = path
                elif trail_index == segment_index:
                    visible_path = path[: point_index + 1]
                else:
                    visible_path = np.empty((0, 3))
                line.set_data(visible_path[:, 0], visible_path[:, 1])
                line.set_3d_properties(visible_path[:, 2])

        frame_count = len(trajectory.servo_angles)
        slider_ax = fig.add_axes([0.17, 0.13, 0.75, 0.035])
        frame_slider = Slider(
            slider_ax,
            "Trajectory point",
            0,
            frame_count - 1,
            valinit=0,
            valstep=1,
            valfmt="%0.0f",
        )
        play_ax = fig.add_axes([0.055, 0.115, 0.09, 0.065])
        play_button = Button(play_ax, "Play", color="#ecf0f1", hovercolor="#d5d8dc")
        frame_state = {"index": 0, "playing": False}
        timer = fig.canvas.new_timer(interval=80)

        def on_slider_changed(value: float) -> None:
            timer.stop()
            frame_state["playing"] = False
            play_button.label.set_text("Play")
            frame_state["index"] = int(value)
            update_frame(frame_state["index"])
            fig.canvas.draw_idle()

        def advance_frame() -> None:
            next_index = frame_state["index"] + 1
            if next_index >= frame_count:
                timer.stop()
                frame_state["playing"] = False
                play_button.label.set_text("Play")
                return

            frame_state["index"] = next_index
            update_frame(next_index)
            frame_slider.eventson = False
            frame_slider.set_val(next_index)
            frame_slider.eventson = True
            fig.canvas.draw_idle()

        def toggle_playback(_event) -> None:
            if frame_state["playing"]:
                timer.stop()
                frame_state["playing"] = False
                play_button.label.set_text("Play")
                return

            if frame_state["index"] >= frame_count - 1:
                frame_state["index"] = 0
                update_frame(0)
                frame_slider.eventson = False
                frame_slider.set_val(0)
                frame_slider.eventson = True
            frame_state["playing"] = True
            play_button.label.set_text("Pause")
            timer.start()

        timer.add_callback(advance_frame)
        frame_slider.on_changed(on_slider_changed)
        play_button.on_clicked(toggle_playback)
        update_frame(0)
        fig._hubart_slider = frame_slider
        fig._hubart_play_button = play_button
        fig._hubart_timer = timer
        fig._hubart_playback_state = frame_state
        fig._hubart_advance_frame = advance_frame

    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Simulate Hubart's IK model and drawing paths.")
    parser.add_argument("--image", type=Path, help="Image file to convert to and animate as a robot path.")
    parser.add_argument("--target", type=float, nargs=3, metavar=("X", "Y", "Z"), help="One target position in meters.")
    args = parser.parse_args()

    if args.image:
        draw_simulation(prepare_image_trajectory(args.image))
        return

    if args.target:
        global TARGET_POSITION
        TARGET_POSITION = tuple(args.target)
        draw_simulation()
        return

    if IMAGE_PATH:
        draw_simulation(prepare_image_trajectory(Path(IMAGE_PATH)))
    else:
        draw_simulation()


if __name__ == "__main__":
    main()