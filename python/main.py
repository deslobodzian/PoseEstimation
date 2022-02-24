import math

import numpy as np
import matplotlib.pyplot as plt


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
    c2 = p.dot(g) + v.dot(v) - (speed * speed)
    c3 = v.dot(g)
    c4 = 0.25 * g.dot(g)
    coeffs = [c0, c1, c2, c3, c4]
    return np.roots(coeffs)

def trajectory(pos, vel, gravity, angle):
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
    target_pos = np.array([1, 1.5, 5])
    ball_pos = np.array([0, 0.0, 0])
    ball_exit_vel = np.array([0.0, 5.0, 20.0])
    target_vel = np.array([0.9, 0.0, 0.0])
    gravity = np.array([0, 0, 9.81])
    speed = np.linalg.norm(ball_exit_vel)
    a1, a2 = calculate_angle(speed, 3, 4, -9.81)
    # tr = pos + (vel * time[0] + (0.5 * gravity * (time[0] * time[0])))
    time = calculate_lead_time(target_pos, target_vel, gravity, speed)
    aim_point = target_pos + target_vel * time[0] + (0.5 * gravity * (time[0] * time[0]))
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    x, y, z = trajectory(ball_pos, ball_exit_vel, gravity, a1)
    ax.scatter(aim_point[0], aim_point[1], aim_point[2])
    ax.plot3D(x, y, z)
    plt.show()
    # print(pos + vel*time[0] + 0.5 * gravity * (time[0] * time[0]))

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
