import math

import numpy as np
import matplotlib.pyplot as plt
import scipy.spatial.transform
from scipy.spatial.transform import Rotation


def angle(pos, goal, speed, g):
    root = math.pow(speed, 4) - (g * (g * pos[0] * pos[0] + 2.0 * pos[2] * speed * speed))
    if root < 0.0:
        return None
    root = math.sqrt(root)
    return math.atan((speed * speed - root) / (g * pos[0]))


def travel_time(x, angle, speed):
    return x / (math.cos(angle) * speed)


def calculate_lead_time(p, v, g, speed):
    c0 = p.dot(p)
    c1 = 2 * p.dot(v)
    c2 = v.dot(v) - (speed * speed)
    coeffs = [c0, c1, c2]
    return np.roots(coeffs)


def cartesian_to_spherical(b):
    root = (b[0] * b[0]) + (b[1]*b[1]) + (b[2] * b[2])
    r = math.sqrt(root)
    theta = math.atan(b[1]/b[0])
    phi = math.atan(math.hypot(b[0], b[1]) / b[2])
    return r, theta, phi


def trajectory(pos, vel, gravity):
    x, y, z = [], [], []
    for t in np.arange(0, 10, 0.1):
        xt = pos[0] + vel[0] * t
        yt = pos[1] + vel[1] * t
        zt = pos[2] + vel[2] * t - (0.5 * gravity[2] * (t * t))
        if (zt < 0):
            break
        x.append(xt)
        y.append(yt)
        z.append(zt)
    return x, y, z

def calculate_angle(v, x, z, g):
    num = (v * v) + math.sqrt(math.pow(v, 4) - g * (g * (x * x) + 2 * z * (v * v)))
    num1 = (v * v) - math.sqrt(math.pow(v, 4) - g * (g * (x * x) + 2 * z * (v * v)))
    denom = g * x
    return math.atan(num / denom), math.atan(num1 / denom)

def main():

    target_pos = np.array([8.2296, 4.1148, 0.220472])
    target_vel = np.array([0.0, 0.9, 0.0])
    ball_pos = np.array([0, 4.1148, 0])

    gravity = np.array([0, 0, 9.81])
    speed = 10.0

    a1, a2 = calculate_angle(speed, target_pos[0], target_pos[2], gravity[2])

    x_vel = speed * math.cos(a1)
    z_vel = speed * math.sin(a1)
    ball_exit_vel = np.array([x_vel, 0.0, z_vel])
    speed = np.linalg.norm(ball_exit_vel)
    print("angle one " + str(a1))
    print("angle two " + str(a2))

    # tr = pos + (vel * time[0] + (0.5 * gravity * (time[0] * time[0])))
    time = calculate_lead_time(target_pos, target_vel, gravity, speed)
    ld = target_pos + (target_vel * time[1])
    print(ld)
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    x, y, z = trajectory(ball_pos, ball_exit_vel, gravity)
    r, theta, phi = cartesian_to_spherical(ld)
    print("r " +str(r))
    print("theta " + str(theta))
    print("phi " + str(phi))
    # new_a = calculate_angle(speed, ld[0], target_pos[2], gravity[2])
    # print(r * math.cos(theta))
    # print(r * math.cos(phi))
    # print(r * math.sin(theta))
    x_new, y_new, z_new = trajectory(ball_pos, ball_exit_vel, gravity)
    ax.scatter(target_pos[0], target_pos[1], target_pos[2], c='r')
    ax.scatter(ld[0], ld[1], ld[2], c='g')
    ax.plot3D(x, y, z)
    ax.plot3D(x_new, y_new, z_new, c='g')
    plt.show()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
