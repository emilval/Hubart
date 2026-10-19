import numpy as np

A4_WIDTH_M = 0.210
A4_HEIGHT_M = 0.297
IMAGE_WIDTH_PX = 1024
IMAGE_HEIGHT_PX = 1024
SCALE_M_PER_PIXEL = min(
    A4_WIDTH_M / IMAGE_WIDTH_PX,
    A4_HEIGHT_M / IMAGE_HEIGHT_PX,
)



x0 = 0.3545 # distance to wall
# y0 and z0 define the position of the top left corner of the page
y0 = 0.0750
z0 = 0.470

# The image's top-left is anchored at (x0, y0, z0) on a plane parallel to YZ.
# Image-right maps toward -Y and image-down maps toward -Z.
def map_2d_to_3d(u, v):
    x = x0 
    y = y0 - u
    z = z0 - v
    return x, y, z


# takes 2d image and maps it to 3d coordinates, returns a list of lists of 3d coordinates
def map_image_to_3d(image):
    image_3d = []
    for stroke in image:
        new_stroke = []
        for point in stroke:
            u, v = point
            u = u * SCALE_M_PER_PIXEL
            v = v * SCALE_M_PER_PIXEL
            x, y, z = map_2d_to_3d(u, v)
            new_stroke.append((x, y, z))
        image_3d.append(new_stroke)
    return image_3d


def add_page_tilt(coordinates, tilt_angle_degrees):
    """Tilt mapped strokes about the page-center line parallel to the Y axis.

    Positive angles offset points lower on the page toward increasing X.
    The coordinates remain in meters, and the page-center height stays fixed.
    """
    if not np.isfinite(tilt_angle_degrees) or abs(tilt_angle_degrees) >= 90:
        raise ValueError("Tilt angle must be finite and between -90 and 90 degrees")

    page_center_z = z0 - A4_HEIGHT_M / 2
    x_offset_per_meter = np.tan(np.radians(tilt_angle_degrees))

    return [
        [
            (x + (page_center_z - z) * x_offset_per_meter, y, z)
            for x, y, z in stroke
        ]
        for stroke in coordinates
    ]

