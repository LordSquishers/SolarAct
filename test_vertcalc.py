import numpy as np

# x,y of panel joints
# top right (1,1,z)
# top left (-1,1,z)
# bottom left (-1,-1,z)
# bottom right (1,-1,z)
xy_pts = np.array([
    [1, 1],
    [-1, 1],
    [-1,-1],
    [1,-1]
])

z_error_margin = 0.1  # meters? anything really
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
    z_offset = -z_pts.min() + z_error_margin  # lowers to safety margin
    z_pts += z_offset

    print('TR, TL, BL, BR')
    print(z_pts)

    # max checking and output of linear act. lengths
    if z_pts.max() > max_actuator_height:
        print('Invalid solution. Max actuator height', z_pts.max(), 'm')
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
for idx in range(4):
    x = xy_pts[idx][0]
    y = xy_pts[idx][0]
    z = zs[idx]
    ps.append([x, y, z])

ps = np.array(ps)
peri = 0  # TODO verify perimeter calculations -- this is to verify pt calculations
for idx in range(4):
    diff = ps[idx] - ps[(idx + 1) % 4]
    print(diff)
    peri += np.linalg.norm(diff)

print(peri)