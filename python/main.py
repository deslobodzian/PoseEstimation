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
    c1 = p.dot(v)
    c2 = (speed * speed) - v.dot(v)

    calc = c1 * c1 + c2 * c0
    t = 0
    if (calc >= 0):
        t = (c1 + math.sqrt(calc)) / c0
        if (t < 0):
            t = 0

    return t


def cartesian_to_spherical(b):
    root = (b[0] * b[0]) + (b[1] * b[1]) + (b[2] * b[2])
    r = math.sqrt(root)
    theta = math.atan(b[1] / b[0])
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
    root = math.pow(v, 4) - g * (g * (x * x) + 2 * z * (v * v))
    if root > 0:
        num = (v * v) + math.sqrt(root)
        num1 = (v * v) - math.sqrt(root)
        denom = g * x
        return math.atan(num / denom), math.atan(num1 / denom)
    return 0, 0


def velocity(height, angle):
    return math.sqrt(2 * 9.81 * height) / math.sin(angle)


def height(vel, angle):
    return ((vel * vel) * math.pow(math.sin(angle), 2)) / (2.0 * 9.81)


def ideal_speed(target, min_h):
    speed = 0
    while speed < 11.0:
        a1, a2 = calculate_angle(speed, target[0], target[2], 9.81)
        height1 = height(speed, a1)
        height2 = height(speed, a2)

        if (min_h < height1 < 3.35) or (min_h < height2 < 3.35):
            print(height1)
            print(height2)
            return speed
        speed = speed + 0.01


def calculate_point(p0, p2):
    temp = p0 + ((p2 - p0) / 2.0)
    return temp + (0.25 * (p2 - p0)[0], 0, 0)

def trajectory_air(vel, angle):
    v_term = 35.0 # mps
    x, y, z = [], [], []
    angle1 = angle
    print(angle)
    speed = vel
    for t in np.arange(0, 10, 0.1):
        xt = ((speed * v_term) / 9.81) * (math.cos(angle1) * (1 - math.exp((-9.81 * t) / v_term)))
        yt = 0
        zt = (v_term / 9.81) * (speed * math.sin(angle1) + v_term) * (1 - math.exp((-9.81 * t) / v_term)) - (v_term * t)
        if (zt < 0):
            break
        x.append(xt)
        y.append(yt)
        z.append(zt)
    return x, y, z



def calculate_air(vel, angle, p):
    v_term = 35.0 # mps
    x, y, z = [], [], []
    angle1 = 0
    print(angle)
    speed = 0
    error2 = p[0] + 0.5
    error1 = p[0] - 0.5
    error3 = p[2] + 0.5
    error4 = p[2] - 0.5
    i = 0
    while speed < 45.0:
        while angle1 < math.pi:
            for t in np.arange(0, 10, 0.1):
                i = 0
                xt = ((speed * v_term) / 9.81) * (math.cos(angle1) * (1 - math.exp((-9.81 * t) / v_term)))
                yt = 0
                zt = (v_term / 9.81) * (speed * math.sin(angle1) + v_term) * (1 - math.exp((-9.81 * t) / v_term)) - (v_term * t)
                # # if (zt < 0):
                #     break
                x.append(xt)
                y.append(yt)
                z.append(zt)
            for x1 in x:
                i = i + 1
                if error1 < x1 < error2:
                    print("point is x: " + str(x1))
                    print("point is z: " + str(z[i]))
                    print("speed is: " + str(speed))
                    print("angle is: " + str(angle1))
                    break
            angle1 = angle1 + 0.01
        speed = speed + 0.01
    # return 0, 0

def main():
    target_pos = np.array([6.3, 0.0, 2.4384])
    target_vel = np.array([0.0, 3.0, 0.0])
    ball_pos = np.array([0, 0, 0])

    height = 3.3528  # meters
    min_height = 2.43  # meters

    p0 = np.array([6, 6, 2.43])
    p1 = np.array([7.2192, 7.2192, 2.43])
    goal = calculate_point(p0, p1)
    target_pos = goal

    speed = ideal_speed(goal, min_height)
    print("Ideal speed is " + str(speed))

    gravity = np.array([0, 0, 9.81])

    a1, a2 = calculate_angle(speed, target_pos[0], target_pos[2], gravity[2])

    x_vel = speed * math.cos(a2)
    z_vel = speed * math.sin(a2)
    ball_exit_vel = np.array([x_vel, 0.0, z_vel])
    x_vel1 = speed * math.cos(a2)
    z_vel1 = speed * math.sin(a2)
    ball_exit_vel1 = np.array([x_vel1, 0.0, z_vel1])
    speed = np.linalg.norm(ball_exit_vel)
    # print("angle one " + str(a1))
    # print("angle two " + str(a2))

    # tr = pos + (vel * time[0] + (0.5 * gravity * (time[0] * time[0])))
    # time = calculate_lead_time(target_pos, target_vel, gravity, speed)
    # ld = target_pos + (target_vel * time)
    # print("Inital target pos" + str(target_pos))
    # print("aim at point" + str(ld))
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    calculate_air(speed, a2, goal)
    # a, v = calculate_air(speed, a1, goal)

    v = speed + (np.linalg.norm(goal) * 0.12)
    a = a2
    x, y, z = trajectory_air(v, a)
    x1, y1, z1 = trajectory(ball_pos, ball_exit_vel1, gravity)
    # r, theta, phi = cartesian_to_spherical(ld)
    # print("r " +str(r))
    # print("theta " + str(theta))
    # print("phi " + str(phi))
    # new_a = calculate_angle(speed, ld[0], target_pos[2], gravity[2])
    # print(r * math.cos(theta))
    # print(r * math.cos(phi))
    # print(r * math.sin(theta))
    # x1 = np.array([ld[0], ld[1]])
    # x2 = np.array([target_pos[0], target_pos[1]])
    # ang = math.acos(x1.dot(x2) / (np.linalg.norm(x1) * np.linalg.norm(x2)))
    # print("angle should be " + str(ang) + "radians")
    # print("angle should be " + str((ang * 180) / math.pi) + "degrees")
    # x_new, y_new, z_new = trajectory(ball_pos, ball_exit_vel, gravity)
    ax.scatter(p0[0], p0[1], p0[2])
    ax.scatter(p1[0], p1[1], p1[2])
    print(goal)
    ax.scatter(goal[0], goal[1], goal[2])
    ax.scatter(target_pos[0], target_pos[1], target_pos[2], c='r')
    # ax.scatter(ld[0], ld[1], ld[2], c='g')
    ax.plot3D(x, y, z)
    ax.plot3D(x1, y1, z1)
    # ax.plot3D(x_new, y_new, z_new, c='g')
    plt.show()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
