import numpy as np
import matplotlib.pyplot as plt

# x,y of panel joints
# top right (1,1,z)
# top left (-1,1,z)
# bottom middle (x,-0.75,z)

# example crate
xy_pts = np.array([
    [40., 15.],
    [-40., 15.],
    [0.,-10.]
])

# square
xy_pts = np.array([
    [1., 1.],
    [-1., 1.],
    [0.,-0.75]
])

z_error_margin = 0.1  # meters? anything really lowest z value
x_step_size = 0.01
max_actuator_height = 1.5  # this is self explanatory (meters)

def calculate_normal(azimuth, elevation):
    # radians to degrees
    azimuth *= np.pi / 180.0
    elevation *= np.pi / 180.0

    # converting azi elev. to unit normal
    x = np.sin(azimuth) * np.cos(elevation)
    y = np.cos(azimuth) * np.cos(elevation)
    z = np.sin(elevation)
    return np.array([x, y, z])


def find_z_adjustable(y, plane_eq, max_xs):
    a = plane_eq[0]
    b = plane_eq[1]
    c = plane_eq[2]
    d = plane_eq[3]

    best_min_x = 9999
    best_max_x = -9999
    x_g = np.arange(max_xs[0], max_xs[1], x_step_size)
    z_g = np.copy(x_g)

    idx = 0
    for x_pt in x_g:
        x_val = x_pt
        z_val = -(a*x_pt + b*y + d) / c
        z_g[idx] = z_val
        idx += 1
        if z_val > z_error_margin and z_val < max_actuator_height:
            best_max_x = max(x_val, best_max_x)
            best_min_x = min(x_val, best_min_x)
    # print('midpoint x', 0, 'mid z', -(a*0 + b*y + d) / c)

    # plt.plot(x_g, z_g)
    # plt.show()

    new_x = (best_max_x + best_min_x) / 2
    new_z = -(a * new_x + b * y + d) / c
    # print(new_x, new_z)
    # print(best_min_x, best_max_x)
    return new_x, new_z

def calculate_lengths(normal):
    unit_normal = normal / normal.sum()
    p0 = np.array([0,0,0.5]) # center point of panel array. (for now)

    # PLANAR MATH(s)
    a = unit_normal[0]
    b = unit_normal[1]
    c = unit_normal[2]
    d = -(a * p0[0] + b * p0[1] + c * p0[2])
    print('Plane Equation:', np.array([a, b, c, d]))

    # EVALUATE AND CORRECT Z COORDS
    z_pts = []
    for point in xy_pts:
        x = point[0]
        y = point[1]
        z = -(a*x + b*y + d) / c  # solve for z

        z_pts.append(z)

    z_pts = np.array(z_pts)  # convert to numpy

    # NEW #
    # adjustable bottom point #
    x, z = find_z_adjustable(xy_pts[2][1], [a, b, c, d], [xy_pts[1][0], xy_pts[0][0]])
    xy_pts[2][0] = x
    z_pts[2] = z

    # if z is less than 0, it means we're clipping into the container.
    # increase the midpoint by the amount we're clipping plus MoE
    # this lowers the whole panel to the minimum to conserve space.
    z_offset = -z_pts.min() + z_error_margin  # lowers to safety margin
    z_pts += z_offset

    ps = []
    for idx in range(len(z_pts)):
        x = xy_pts[idx][0]
        y = xy_pts[idx][1]
        z = z_pts[idx]
        ps.append([x, y, z])
        idx += 1
    ps = np.array(ps)

    print('Points:')
    print(ps)

    # max checking and output of linear act. lengths
    if z_pts.max() > max_actuator_height:
        print('Invalid solution. Max actuator height', round(z_pts.max(), 3), 'm')
        return z_pts
    else:
        return z_pts


# inputs in degrees (NYC at Noon, march 29th 2021)
azimuth = 250.07
elevation = 60

n = calculate_normal(azimuth, elevation)
# n = np.array([0, 0, 1])
zs = calculate_lengths(n)

# issues:
# large rectangular panels have a hard time tracking degrees < 80
# max height?
