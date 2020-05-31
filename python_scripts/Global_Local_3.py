"""
Dijkstra grid based planning

author: Atsushi Sakai(@Atsushi_twi)
"""

import matplotlib.pyplot as plt
import math
import numpy as np
import sys
sys.path.append("../../")

show_animation = True


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        #self.max_speed = 1.0  # [m/s]
        self.max_speed = 0.5  # [m/s]
        #self.min_speed = -0.5  # [m/s]
        self.min_speed = -0.5  # [m/s]
        #self.max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
        #self.max_accel = 0.2  # [m/ss]
        self.max_accel = 0.2    # [m/ss]
        #self.max_dyawrate = 40.0 * math.pi / 180.0  # 2.5[rad/ss]
        self.max_dyawrate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.005  # [m/s]
        #self.v_reso = 0.05  # [m/s]
        #self.yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
        self.yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
        #self.dt = 0.1  # [s]
        self.dt = 0.08  # [s]
        #self.predict_time = 3.0  # [s]
        self.predict_time = 2.0  # [s]
        #self.to_goal_cost_gain = 1.0
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 2.0
        #self.speed_cost_gain = 1.5
        #self.robot_radius = 1.0  # [m]
        self.robot_radius = 0.9  # [m]


#DWA

def motion(x, u, dt):
    # motion model

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def calc_trajectory(xinit, v, y, config):

    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    return traj


def calc_final_input(x, u, dw, config, goal, ob):

    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0
    best_traj = np.array([x])

    # evalucate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):
            traj = calc_trajectory(xinit, v, y, config)

            # calc cost
            to_goal_cost = calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])
            ob_cost = calc_obstacle_cost(traj, ob, config)
            # print(ob_cost)

            final_cost = to_goal_cost + speed_cost + ob_cost

            #print (final_cost)

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, y]
                best_traj = traj

    return min_u, best_traj


def calc_obstacle_cost(traj, ob, config):
    # calc obstacle cost inf: collistion, 0:free

    skip_n = 2
    minr = float("inf")

    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)
            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal, config):
    # calc to goal cost. It is 2D norm.

    goal_magnitude = math.sqrt(goal[0]**2 + goal[1]**2)
    traj_magnitude = math.sqrt(traj[-1, 0]**2 + traj[-1, 1]**2)
    dot_product = (goal[0] * traj[-1, 0]) + (goal[1] * traj[-1, 1])
    error = dot_product / (goal_magnitude * traj_magnitude)
    error_angle = math.acos(error)
    cost = config.to_goal_cost_gain * error_angle

    return cost


def dwa_control(x, u, config, goal, ob):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u, traj = calc_final_input(x, u, dw, config, goal, ob)

    return u, traj


def plot_arrow(x, y, yaw, length=0.5, width=0.1):
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

#DIJKSTRA

def dijkstra_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]
        #  print("current", current)

        # show graph
        if show_animation:
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(motion)):
            node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry


def calc_final_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if obmap[int(node.x)][int(node.y)]:
        return False

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x > maxx:
        return False
    elif node.y > maxy:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))


    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)


    # obstacle map generation
    obmap = [[False for i in range(int(xwidth))] for i in range(int(ywidth))]
    for ix in range(int(xwidth)):
        x = ix + minx
        for iy in range(int(ywidth)):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def main():
    print(__file__ + " start!!")


    #Dijkstra
    # start and goal position
    sx = 1.0  # [m]
    sy = 1.0  # [m]
    gx = 8.0  # [m]
    gy = 8.0  # [m]
    grid_size = 1.0  # [m]
    robot_size = 1.0  # [m]

    ox = [5]
    oy = [5]

    for i in range(12):
        ox.append(i)
        oy.append(0.0)
    for i in range(12):
        ox.append(12.0)
        oy.append(i)
    for i in range(13):
        ox.append(i)
        oy.append(12.0)
    for i in range(13):
        ox.append(0.0)
        oy.append(i)
    """for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)"""

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = dijkstra_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)
    rx.reverse()
    ry.reverse()
    print("The path is:")
    print ("rx", rx)
    print ("ry", ry)
    #New code
    a = len(rx)
    x = np.array([1.0, 1.0, math.pi / 8.0, 0.0, 0.0])
    i= 0
    ob = np.array([[4, 2],
                   [5, 4],
                   [5.0, 5.0],
                   [5.0, 6.0],
                   [5.0, 7.0],
                   [5.0, 8.0],
                   [5.0, 9.0]
                   ])
    u = np.array([0.0, 0.0])
    config = Config()
    traj = np.array(x)
    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")
    while i < a:
        goal = np.array([rx[i], ry[i]])
        print("goal",goal)
        print("i",i)
        goal_reached = False
        follow_dijkstra = False
        #for i in range(1000):
        while(not goal_reached):
            u, ltraj = dwa_control(x, u, config, goal, ob)
            x = motion(x, u, config.dt)
            traj = np.vstack((traj, x))  # store state history
            for o in ob:
                    if (o[0] == goal[0]) and (o[1] == goal[1]):
                        print(goal)
                        goal_reached = True
            if show_animation:
                plt.cla()
                plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
                plt.plot(x[0], x[1], "xr")
                plt.plot(ob[:, 0], ob[:, 1], "om")
                plt.plot(goal[0], goal[1], "xb")
                plot_arrow(x[0], x[1], x[2])
                plt.plot(rx, ry, "-r")
                plt.axis("equal")
                plt.grid(True)
                plt.pause(0.0001)
            if show_animation:
                plt.plot(ox, oy, ".k")
                plt.plot(rx, ry, "-r")
            if follow_dijkstra:
                if math.sqrt((x[0] -  goal[0])**2 + (x[1] - goal[1])**2) <= config.robot_radius:
                    print("goal[0]",goal[0])
                    print("goal[1]",goal[1])
                    print("Goal!!")
                    goal_reached = True
            else:
                if math.sqrt((x[0] - rx[-1])**2 + (x[1] - ry[-1])**2) <= config.robot_radius:
                    print("rx-1",rx[-1])
                    print("ry-1",ry[-1])
                    print("Goal!!")
                    break
        i+=1
        goal_reached = False


    print("Done")
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-c")
        plt.pause(0.0001)
    if show_animation:
        plt.plot(rx, ry, "-r")
        #plt.show()
    plt.show()


if __name__ == '__main__':
    main()
