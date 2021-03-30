import numpy as np
import matplotlib.pyplot as plt

# x,y of panel joints
# top right (1,1,z)
# top left (-1,1,z)
# bottom middle (x,-0.5,z)
xy_pts = np.array([
    [1, 1],
    [-1, 1],
    [0,-0.5]
])

z_error_margin = 0.1  # meters? anything really
x_margin = 0.05
x_step_size = 0.01
max_actuator_height = 1  # this is self explanatory

def calculate_normal(azimuth, elevation):
    # radians to degrees
    azimuth *= np.pi / 180.0
    elevation *= np.pi / 180.0

    # converting azi elev. to unit normal
    x = np.sin(azimuth) * np.cos(elevation)
    y = np.cos(azimuth) * np.cos(elevation)
    z = np.sin(elevation)
    return np.array([x, y, z])


def find_z_adjustable(xy, z, plane_eq, max_xs):
    a = plane_eq[0]
    b = plane_eq[1]
    c = plane_eq[2]
    d = plane_eq[3]
    y = xy[1]

    best_min_x = 9999
    best_max_x = -9999
    x_g = np.arange(-1, 1, 0.01)  # np.arange(max_xs[0], max_xs[1], x_step_size)
    print(x_g.min())
    z_g = x_g

    idx = 0
    for x_pt in x_g:
        x_val = x_pt
        z_val = -(a*x_pt + b*y + d) / c
        z_g[idx] = z_val
        idx += 1
        if z_val > z_error_margin and z_val < max_actuator_height:
            best_max_x = max(x_val, best_max_x)
            best_min_x = min(x_val, best_min_x)

    print('min x', best_min_x, 'min z',-(a*best_min_x + b*y + d) / c, 'max x', best_max_x, 'max z', -(a*best_max_x + b*y + d) / c)
    print('midpoint x', 0.5, 'mid z', -(a*0.5 + b*y + d) / c)

    plt.plot(x_g, z_g)
    print(x_g.min())
    plt.show()


def calculate_lengths(normal):
    unit_normal = normal / normal.sum()
    p0 = np.array([0,0,0.5]) # center point of panel array. (for now)

    # PLANAR MATH(s)
    a = unit_normal[0]
    b = unit_normal[1]
    c = unit_normal[2]
    d = -(a * p0[0] + b * p0[1] + c * p0[2])
    print('eq:', np.array([a, b, c, d]))

    # EVALUATE AND CORRECT Z COORDS
    z_pts = []
    for point in xy_pts:
        x = point[0]
        y = point[1]
        z = -(a*x + b*y + d) / c  # solve for z

        z_pts.append(z)

    # if z is less than 0, it means we're clipping into the container.
    # increase the midpoint by the amount we're clipping plus MoE
    z_pts = np.array(z_pts)  # convert to numpy
    # z_offset = -z_pts.min() + z_error_margin  # lowers to safety margin
    # z_pts += z_offset

    # NEW #
    # adjustable bottom point #
    z_bot = find_z_adjustable(xy_pts[2], z_pts[2], [a, b, c, d], [xy_pts[1][0], xy_pts[0][0]])

    print('TR, TL, BMID')
    print(z_pts)

    # max checking and output of linear act. lengths
    if z_pts.max() > max_actuator_height:
        print('Invalid solution. Max actuator height', round(z_pts.max(), 3), 'm')
        return z_pts
    else:
        return z_pts


# inputs in degrees (NYC at Noon, march 29th 2021)
azimuth = 155.82
elevation = 50.45

n = calculate_normal(azimuth, elevation)
# n = np.array([0, 0, 1])
zs = calculate_lengths(n)

ps = []
for idx in range(len(xy_pts)):
    x = xy_pts[idx][0]
    y = xy_pts[idx][0]
    z = zs[idx]
    ps.append([x, y, z])

ps = np.array(ps)
peri = 0  # TODO verify perimeter calculations -- this is to verify pt calculations
for idx in range(len(xy_pts)):
    diff = ps[idx] - ps[(idx + 1) % len(xy_pts)]
    # print(diff)
    peri += np.linalg.norm(diff)

print('Permieter', round(peri,3), 'm')