import numpy as np

#1024x2024
scale = 0.297 / 2024 #scale image to fit on a4

x0 = 0.355 # distance to wall
# y0 and z0 define the position of the top left corner of the page
y0 = 0.05 
z0 = 0.46

# this function assumes that the plane is parallel to the yz-plane
def map_2d_to_3d(u, v):
    x = x0
    y = y0 + u
    z = z0 + v
    return x, y, z


# takes 2d image and maps it to 3d coordinates, returns a list of lists of 3d coordinates
def map_image_to_3d(image):
    image_3d = []
    for stroke in image:
        new_stroke = []
        for point in stroke:
            u, v = point
            u = u * scale
            v = v * scale
            x, y, z = map_2d_to_3d(u, v)
            new_stroke.append((x, y, z))
        image_3d.append(new_stroke)
    return image_3d

