"""
Dijkstra grid based planning

author: Atsushi Sakai(@Atsushi_twi)
"""

import matplotlib.pyplot as plt
import math

show_animation = True


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def dijkstra_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]pytho
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1) #start
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1) #goal
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart #openset[indice_init]=nstart

    while 1:
        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]
        #c_id = min(openset, key=lambda o: openset[o].cost) #de todo lo que hay en openset, busca el id con el minimo coste para empezar con el
        #current = openset[c_id] #la posicion actual pasa a ser la que tiene el minimo coste
        #  print("current", current)

        # show graph
        if show_animation: #muestra la posicion actual
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y: #si la posicion actual es igual que el goal, para si no no
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id] #quita el elemento del openset
        # Add it to the closed set
        closedset[c_id] = current #pone el elemento en el closedset

        # expand search grid based on motion model # En funcion del modelo de motion, se expande la busqeuda del grid
        for i in range(len(motion)): #busca el siguiente punto a trataro
            node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny) #busca el indice donde esta

            if not verify_node(node, obmap, minx, miny, maxx, maxy): #verifica que la posicion del nodo este dentro de la grid
                continue

            if n_id in closedset: #el punto ya ha estado tratado previamente
                continue
            # Otherwise if it is already in the open set
            if n_id in openset: # si el punto no ha sido tratado (esta en el openset)
                if openset[n_id].cost > node.cost: #comprueba que el coste sea mayor que el nodo actual
                    openset[n_id].cost = node.cost  # si es asi, le anade el nuevo coste al nodo
                    openset[n_id].pind = c_id #le anade el indice
            else:
                openset[n_id] = node #si no esta en el openset, lo anade

    rx, ry = calc_final_path(ngoal, closedset, reso) #calcula el path final con el goal, el closeset

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

    minx = round(min(ox)) #minimum obstacle x
    miny = round(min(oy)) #minimum obstacle y
    maxx = round(max(ox)) #maximum obstacle x
    maxy = round(max(oy)) #maximum obstacle y


    xwidth = round(maxx - minx) #
    ywidth = round(maxy - miny)


    # obstacle map generation
    obmap = [[False for i in range(int(xwidth))] for i in range(int(ywidth))]
    for ix in range(int(xwidth)):
        x = ix + minx # sum the minimum of x
        for iy in range(int(ywidth)):
            y = iy + miny
            #  print(x, y) #sum the minimum of y
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

    # start and goal position
    sx = 5.0 #10.0  # [m]
    sy = 30.0 #10.0  # [m]
    gx = 50.0 #50.0  # [m]
    gy = 10.0 #50.0  # [m]
    grid_size = 0.8 #1.0  # [m]
    robot_size = 5.0  # [m]

    ox = [10,16,25,31,42]
    oy = [15,37,44,28,30]

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = dijkstra_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)

    print("The path is:")
    print ("rx", rx)
    print ("ry", ry)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()
