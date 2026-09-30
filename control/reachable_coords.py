import numpy as np
import matplotlib.pyplot as plt

cos = np.cos
sin = np.sin
def rotmat_x(theta):
    c = cos(theta)
    s = sin(theta)
    matrix = np.array([
        [1,  0,  0, 0],
        [0,  c, -s, 0],
        [0,  s,  c, 0],
        [0,  0,  0, 1]])
    return matrix

def rotmat_z(theta):
    c = cos(theta)
    s = sin(theta)
    matrix = np.array([
        [c, -s,  0, 0],
        [s,  c,  0, 0],
        [0,  0,  1, 0],
        [0,  0,  0, 1]])
    return matrix

def transmat(dx, dy, dz):
    matrix = np.array([
        [1, 0, 0, dx],
        [0, 1, 0, dy],
        [0, 0, 1, dz],
        [0, 0, 0,  1]])
    return matrix

def fk(theta1, theta2, theta3, translation, alpha):
    x1, y1, z1 = translation['trans1']
    x2, y2, z2 = translation['trans2']
    x3, y3, z3 = translation['trans3']
    alpha = np.radians(alpha) 
    T_world_0 = transmat(0, 0, L1)
    T_0_a = rotmat_z(theta1)
    #print("T_0_a:")
    #print(T_0_a)
    T_a_b = transmat(x1, y1, z1)
    #print("T_a_b:")
    #print(T_a_b)
    T_b_1 = rotmat_x(alpha)
    #print("T_b_1:")
    #print(T_b_1)
    T_1_c = rotmat_z(theta2)
    #print("T_1_c:")
    #print(T_1_c)
    T_c_2 = transmat(x2, y2, z2)
    #print("T_c_2:")
    #print(T_c_2)
    T_2_d = rotmat_z(theta3)
    #print("T_2_d:")
    #print(T_2_d)
    T_d_3 = transmat(x3, y3, z3)
    #print("T_d_3:")
    #print(T_d_3)
    T_0_1 = T_0_a @ T_a_b @ T_b_1
    #print("T_0_1:")
    #print(T_0_1)
    T_0_2 = T_0_1 @ T_1_c @ T_c_2
    #print("T_0_2:")
    #print(T_0_2)
    T_0_3 = T_0_2 @ T_2_d @ T_d_3
    #print("T_0_3:")
    #print(T_0_3)

    p_ee = (T_world_0 @ T_0_3)[:3, 3]

    return p_ee


joint_lim = np.radians(((-90, 90), (-90, 90), (-90, 90)))

L1 = 0.055 
L2 = 0.315
L3 = 0.045
L4 = 0.108
L5 = 0.005
L6 = 0.034
L7 = 0.015
L8 = 0.088
L9 = 0.204

translations = {
    'trans1': (L6, -L4, L2+L3),    # from coordinate frame a to frame 1       
    'trans2': (L7, -L8, -L5),      # from coordinate frame c to frame 2
    'trans3': (0, -L9, 0),         # from coordinate frame d to frame 3
}

alpha = 90


def best_a4_rectangle(projected_points, width, height):
    best_count = 0
    best_start = None
    x_values = projected_points[:, 0]
    y_values = projected_points[:, 1]
    x_starts = np.unique(np.concatenate((x_values, x_values - width)))

    for x_start in x_starts:
        in_x = (x_values >= x_start) & (x_values <= x_start + width)
        sorted_y = np.sort(y_values[in_x])
        if sorted_y.size == 0:
            continue
        counts = np.searchsorted(sorted_y, sorted_y + height, side="right") - np.arange(sorted_y.size)
        index = np.argmax(counts)
        if counts[index] > best_count:
            best_count = int(counts[index])
            best_start = (x_start, sorted_y[index])

    return best_count, best_start


def find_best_a4_plane(points, tolerance=0.005, offset_step=0.0005):
    best = {"count": 0}
    axis_pairs = ((0, 1),)
    a4_sides = (0.210, 0.297)

    for first_axis, second_axis in axis_pairs:
        normal_axis = next(axis for axis in range(3) if axis not in (first_axis, second_axis))
        sorted_indices = np.argsort(points[:, normal_axis])
        sorted_normal = points[sorted_indices, normal_axis]
        offsets = np.arange(
            sorted_normal[0] - tolerance,
            sorted_normal[-1] + tolerance + offset_step,
            offset_step,
        )

        for offset in offsets:
            start = np.searchsorted(sorted_normal, offset - tolerance, side="left")
            stop = np.searchsorted(sorted_normal, offset + tolerance, side="right")
            nearby_points = points[sorted_indices[start:stop]]
            if nearby_points.size == 0:
                continue

            projected = nearby_points[:, (first_axis, second_axis)]
            for width, height in (a4_sides, a4_sides[::-1]):
                count, rectangle_start = best_a4_rectangle(projected, width, height)
                if count <= best["count"]:
                    continue

                first_start, second_start = rectangle_start
                corners = np.zeros((4, 3))
                corners[:, normal_axis] = offset
                corners[:, first_axis] = (
                    first_start, first_start + width, first_start + width, first_start
                )
                corners[:, second_axis] = (
                    second_start, second_start, second_start + height, second_start + height
                )
                best = {
                    "count": count,
                    "corners": corners,
                    "axes": (first_axis, second_axis, normal_axis),
                    "offset": offset,
                }

    return best


sampled_angles = np.random.uniform(
    joint_lim[:, 0], joint_lim[:, 1], size=(50_000, 3)
)
positions = np.array([fk(angles[0], angles[1], angles[2], translations, alpha) for angles in sampled_angles])
best_plane = find_best_a4_plane(positions)
axis_names = "XYZ"
first_axis, second_axis, normal_axis = best_plane["axes"]
print(f"Best A4 plane: {axis_names[first_axis]}{axis_names[second_axis]} plane")
print(f"Points within +/-5 mm: {best_plane['count']} / {len(positions)}")
print("Plane corners (XYZ, m):")
for corner in best_plane["corners"]:
    print("  ", np.array2string(corner, precision=4))

figure = plt.figure()
axes = figure.add_subplot(111, projection="3d")
scatter = axes.scatter(
    positions[:, 0], positions[:, 1], positions[:, 2],
    c=positions[:, 2], s=4, alpha=0.3,
)
corners = best_plane["corners"]
first_values = np.linspace(corners[:, first_axis].min(), corners[:, first_axis].max(), 2)
second_values = np.linspace(corners[:, second_axis].min(), corners[:, second_axis].max(), 2)
first_grid, second_grid = np.meshgrid(first_values, second_values)
plane_grid = [np.full_like(first_grid, best_plane["offset"]) for _ in range(3)]
plane_grid[first_axis] = first_grid
plane_grid[second_axis] = second_grid
axes.plot_surface(*plane_grid, color="tab:orange", alpha=0.35, edgecolor="black", linewidth=0.8)
closed_corners = np.vstack((corners, corners[0]))
axes.plot(*closed_corners.T, color="black", linewidth=2, label="Best-fit A4 plane")
on_plane = (
    (np.abs(positions[:, normal_axis] - best_plane["offset"]) <= 0.005)
    & (positions[:, first_axis] >= corners[:, first_axis].min())
    & (positions[:, first_axis] <= corners[:, first_axis].max())
    & (positions[:, second_axis] >= corners[:, second_axis].min())
    & (positions[:, second_axis] <= corners[:, second_axis].max())
)
axes.scatter(*positions[on_plane].T, color="red", s=12, depthshade=False, label="Points within +/-5 mm")
axes.legend()

axes.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)", title="Sampled Reachable Workspace")
figure.colorbar(scatter, ax=axes, label="Z (m)")
plt.show()