import math

import numpy as np
import utm


def get_distance(wp1, wp2):
    p = (wp1.x, wp1.y)
    q = (wp2.x, wp2.y)
    return math.dist(p,q)    

# For every adjacent points within the same "step": interpolate
def interpolate(wp1, wp2) -> [tuple]:

    # convert wp1, wp2 to x,y
    x1, y1, zone_num, zone_letter = utm.from_latlon(wp1[0], wp1[1])
    x2, y2, _, _                  = utm.from_latlon(wp2[0], wp2[1])

    # interpolate the center line between source merge end and target merging
    wp1_array = np.array((x1, y1))
    wp2_array = np.array((x2, y2))
    interpolate_distance = np.linalg.norm(wp1_array - wp2_array)
    # print(f"interpolate dist: {interpolate_distance}")
    inter_array = np.linspace([x1, y1], [x2, y2], int(interpolate_distance) + 1)
    inter_points_list = []
    for wpt in inter_array:
        # Conver x,y back to lat, lng
        lat, lng = utm.to_latlon(wpt[0], wpt[1], zone_num, zone_letter)
        inter_points_list.append((lat, lng))

    # Return a list of point (list of [x,y])
    # Note, remember to remove the first and last point
    return inter_points_list[1:-1]

# For every adjacent paths within the same "leg": get spline
def get_spline(x0, x1, y0, y1, theta0, theta1, steps=100, t=0):
    # print(f"x0 {x0}, x1 {x1}, y0 {y0}, y1 {y1}, \ntheta0 {theta0}, theta1 {theta1}, steps {steps}, t {t}")
    if np.size(t) == 1:
        t = np.linspace(0, 1, steps)  # np.asarray([1, 5, 10]) #

    # SCALING CHANGES THE RESULT
    ######################### scale for better curves ############################
    dx = x1 - x0
    dy = y1 - y0

    X0 = x0 + 0
    Y0 = y0 + 0

    x0 = 0
    y0 = 0

    if np.abs(dx) < np.abs(dy):  # y changes more than x, so y should be 1
        # remove dy
        s = np.abs(dy) * 1.5
        x1 = (dx + 0.0) / s
        y1 = (dy + 0.0) / s

    else:
        # remove dx
        s = np.abs(dx) * 1.5
        x1 = (dx + 0.0) / s
        y1 = (dy + 0.0) / s
    ################################################################################

    # X = T*b
    # x  =   a*t**3 + b*t**2 + c*t + d
    # x' = 3*a*t**2 + 2*b*t  + c
    # x0 = 0
    # y0 = 0
    # theta0 = 0
    dx0 = np.cos(theta0)  # = c  # all change is at along x at start
    dy0 = np.sin(theta0)
    # print("d0",dx0, dy0)
    # x1 = 2
    # y1 = 1
    # theta1 = 0+np.random.uniform(0.001, 0.01)
    dx1 = np.cos(theta1)  # = c  # all change is at along x at start
    dy1 = np.sin(theta1)
    # print("d1",dx1, dy1)
    t0 = 0
    t1 = 1

    # TREAT X AND Y SEPARATE, SO THERE'S NOT A SINGULARITY

    Ax = np.asarray(
        [
            [1, t0, t0**2, t0**3],  # x  @ 0
            [0, 1, 2 * t0, 3 * t0**2],  # x' @ 0
            [1, t1, t1**2, t1**3],  # x  @ 1
            [0, 1, 2 * t1, 3 * t1**2],
        ]
    )  # x' @ 1

    X = np.asarray([x0, dx0, x1, dx1]).transpose()
    bx = np.linalg.solve(Ax, X)

    Ay = np.asarray(
        [
            [1, t0, t0**2, t0**3],  # x  @ 0
            [0, 1, 2 * t0, 3 * t0**2],  # x' @ 0
            [1, t1, t1**2, t1**3],  # x  @ 1
            [0, 1, 2 * t1, 3 * t1**2],
        ]
    )  # x' @ 1
    Y = np.asarray([y0, dy0, y1, dy1]).transpose()
    by = np.linalg.solve(Ay, Y)

    x = np.dot(np.vstack([np.ones_like(t), t, t**2, t**3]).transpose(), bx)
    y = np.dot(np.vstack([np.ones_like(t), t, t**2, t**3]).transpose(), by)

    x = X0 + x * s
    y = Y0 + y * s

    return x, y

def one_meter_spacing(traj, spacing=1.0):
    # distance = traj[:,]
    traj = np.asarray(traj)
    # print ("traj", np.shape(traj))
    num_pts = len(traj[:, 0])
    distance = np.sqrt((traj[:-1, 0] - traj[1:, 0]) ** 2 + (traj[:-1, 1] - traj[1:, 1]) ** 2)
    cumdist = np.cumsum(distance)

    # print(f"traj {traj}\ndistance {distance}\ncumdist {cumdist}")
    x = [traj[0, 0]]
    y = [traj[0, 1]]
    try:
        z = [traj[0, 2]]
    except:
        z = [0]

    target = spacing + 0.0
    # print(distance, cumdist)
    i = 0
    # print("---------------",np.shape(cumdist), np.shape(distance) )
    while 1 == 1:
        while target > cumdist[i]:
            i += 1
            if i >= len(cumdist):
                break
        if i >= len(cumdist):
            break

        # print(i, distance, cumdist)
        if i == 0:
            excess = target
        else:
            excess = target - cumdist[i - 1]
        eps = 0.0001
        ratio = excess / (distance[i] + eps)
        # print(i, ":", ratio, excess, distance[i], traj[i,0], traj[i+1,0])
        x.append((1 - ratio) * traj[i, 0] + (ratio) * traj[i + 1, 0])
        y.append((1 - ratio) * traj[i, 1] + (ratio) * traj[i + 1, 1])
        try:
            z.append((1 - ratio) * traj[i, 2] + (ratio) * traj[i + 1, 2])
        except:
            z.append(0)
        target += spacing

    # print("one meter:",x)
    if len(x) == 1:
        # print(x[0])
        x = x[0] * np.ones(num_pts)
        y = y[0] * np.ones(num_pts)
        z = z[0] * np.ones(num_pts)
        # print("1.", x)
    else:
        while len(x) < num_pts:  # repeat the last command
            x.append(x[-1])
            y.append(y[-1])
            z.append(z[-1])
        # print("2::",x)
    traj = np.vstack([np.asarray(x), np.asarray(y), np.asarray(z)]).transpose()
    # print ("traj", np.shape(traj))
    return traj

def connect_paths(path1, path2):
    # print(f"path1: \n{path1}")
    # print(f"path2: \n{path2}")

    # plt.figure()
    # plt.plot(path1[:, 0], path1[:, 1], ".r")
    # plt.plot(path2[:, 0], path2[:, 1], ".b")
    # plt.show()

    # This function takes two center lines (paths) as [pts x [x,y]]
    # returns a curved path connecting them for the purposes of
    # completing an intersection
    # INPUT : path1 = [pts x [x,y]]
    # 		  path2 = [pts x [x,y]]
    # OUTPUT: connecting_path = [pts x [x,y]]
    #

    # FIND CLOSEST POINTS
    # from from scipy.spatial.distance import cdist could be used
    # in the general case, but we'll assume we're connecting end points
    dist00 = np.sum((path1[0, :] - path2[0, :]) ** 2)
    dist0e = np.sum((path1[0, :] - path2[-1, :]) ** 2)
    diste0 = np.sum((path1[-1, :] - path2[0, :]) ** 2)
    distee = np.sum((path1[-1, :] - path2[-1, :]) ** 2)

    # CREATE CURVE
    dists = np.asarray([dist00, dist0e, diste0, distee])
    s_id = np.argmin(dists)

    if s_id == 0:  # Imagine an Arrow flowing from path1 to path2
        # print('case 00')
        id0a = 1  # previous point
        id0b = 0
        id1a = 0
        id1b = 1
        id1 = 0
    elif s_id == 1:
        # print('case 0e')
        id0a = 1  # previous point
        id0b = 0
        id1a = -1
        id1b = -2
        id1 = -1
    elif s_id == 2:
        # print('case e0')
        id0a = -2  # previous point
        id0b = -1
        id1a = 0
        id1b = 1
        id1 = 0
    else:
        # print('case ee')
        id0a = -2  # previous point
        id0b = -1
        id1a = -1
        id1b = -2
        id1 = -1

    # GET ANGLES
    vec0 = path1[id0b, :] - path1[id0a, :]
    vec1 = path2[id1b, :] - path2[id1a, :]
    theta0 = np.arctan2(vec0[1], vec0[0])
    theta1 = np.arctan2(vec1[1], vec1[0])
    # print(theta0, theta1)

    # DETERMINE POINTS AND SPACING
    separation = np.sqrt(np.sum((vec0**2)))
    straight_dist = dists[s_id]
    # not sure how to do this in the general case
    heuristic_dist = straight_dist * np.pi / 2

    pts = int(np.ceil(heuristic_dist / separation))

    # GET SPLINE
    x, y = get_spline(path1[id0b, 0], path2[id1, 0], path1[id0b, 1], path2[id1, 1], theta0, theta1, steps=pts)
    # x,y = get_clothoid(path1[id0b,0], path2[id1,0], path1[id0b,1], path2[id1,1], theta0, theta1, steps = pts)

    # print(f"x {x} \ny {y}")

    connecting_path = np.vstack([x, y]).transpose()

    if len(connecting_path) >=2:    
        connecting_path = one_meter_spacing(connecting_path, 1)

    return connecting_path

