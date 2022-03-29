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
    v_term = 35.0  # mps
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


def catapult_calc(spring_disp, angle_disp):
    arm_length = 20.9 * (2.54 / 100)
    arm_mass = 0.56 # kg
    arm_inertia = (1 / 3) * (arm_length * arm_length) * arm_mass
    ball_inertia = (2 / 3) * 0.27 * (0.24 * 0.24)
    inertia = arm_inertia + ball_inertia
    small_arm_length = 6.0 / 39.37
    spring_rate = 2 * (11.0 / 0.0057101471627692) # N per Meter
    spring_force = spring_disp * spring_rate
    torque = spring_force * small_arm_length
    arm_accel = torque / inertia
    exit_angle = angle_disp - math.radians(24.5)
    time = math.sqrt((2.0 / arm_accel) * angle_disp)
    angle_vel = angle_disp / time
    exit_vel = angle_vel * arm_length
    return exit_vel, exit_angle


def main():
    target_pos = np.array([4.5, 0.0, 2.4384])
    target_vel = np.array([1.0, 0.5, 0.0])
    ball_pos = np.array([0, 0, 0])

    speed, angle = catapult_calc(0.0785, math.radians(55))
    print(speed)
    print(angle)
    height = 3.3528  # meters
    min_height = 2.43  # meters

    # p0 = np.array([6, 6, 2.43])
    # p1 = np.array([7.2192, 7.2192, 2.43])
    # goal = calculate_point(p0, p1)
    # target_pos = goal

    # speed = ideal_speed(goal, min_height)
    # print("Ideal speed is " + str(speed))

    gravity = np.array([0, 0, 9.81])

    a1, a2 = calculate_angle(speed, target_pos[0], target_pos[2], gravity[2])
    print(a2)

    x_vel = speed * math.cos(a2)
    z_vel = speed * math.sin(a2)
    ball_exit_vel = np.array([x_vel, 0.0, z_vel])
    # x_vel1 = speed * math.cos(a2)
    # z_vel1 = speed * math.sin(a2)
    # ball_exit_vel1 = np.array([x_vel1, 0.0, z_vel1])
    # speed = np.linalg.norm(ball_exit_vel)
    # print("angle one " + str(a1))
    # print("angle two " + str(a2))

    # tr = pos + (vel * time[0] + (0.5 * gravity * (time[0] * time[0])))
    time = calculate_lead_time(target_pos, target_vel, gravity, speed)
    ld = target_pos + (target_vel * time)
    print("Inital target pos" + str(target_pos))
    print("aim at point" + str(ld))
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    # calculate_air(speed, a2, goal)
    # a, v = calculate_air(speed, a1, goal)

    # v = speed + (np.linalg.norm(goal) * 0.12)
    # a = a2
    x, y, z = trajectory(ball_pos, ball_exit_vel, gravity)
    # r, theta, phi = cartesian_to_spherical(ld)
    # print("r " +str(r))
    # print("theta " + str(theta))
    # print("phi " + str(phi))
    new_speed = speed + np.linalg.norm(target_vel)
    print(new_speed)
    new_a1, new_a2 = calculate_angle(new_speed, ld[0], ld[2], gravity[2])
    x_vel = new_speed * math.cos(new_a2)
    z_vel = new_speed * math.sin(new_a2)
    ball_exit_vel = np.array([x_vel, 0.0, z_vel])
    print(new_a1)
    print(new_a2)
    print("dist to new is " + str(np.linalg.norm(ld)))
    x_new, y_new, z_new = trajectory(ball_pos, ball_exit_vel, gravity)
    # ax.scatter(goal[0], goal[1], goal[2])
    ax.scatter(target_pos[0], target_pos[1], target_pos[2], c='r')
    ax.scatter(ld[0], ld[1], ld[2], c='g')
    ax.plot3D(x, y, z)
    # ax.plot3D(x1, y1, z1)
    ax.plot3D(x_new, y_new, z_new, c='g')
    plt.show()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
