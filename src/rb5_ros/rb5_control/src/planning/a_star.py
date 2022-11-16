import numpy as np
import matplotlib.pyplot as plt
import math
from PIL import Image

from copy import deepcopy
from .priority_queue import PriorityQueue
from .node import Node
from .map import Map

ROBOT_WIDTH = 0.25 # cell width
ROBOT_HEIGHT = 0.25 # cell height

def h(n, g):
    """
    n and g are of type Node
    returns euclidean distance in meters between two coordinates in map grid
    """
    y1 = n.get_y()
    x1 = n.get_x()

    y2 = g.get_y()
    x2 = g.get_x()

    y1 *= ROBOT_HEIGHT
    y2 *= ROBOT_HEIGHT

    x1 *= ROBOT_WIDTH
    x2 *= ROBOT_WIDTH

    d = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    return d

def a_star_search(start, goal, mapp):
    """start and goal are both of type Node"""
    open_lst = PriorityQueue()
    close_lst = []

    open_lst.put((start.f, start))

    while not open_lst.isEmpty():
        f_n, n = open_lst.get()
        succs = mapp.get_neighbors(n)
        for suc in succs:
            suc.set_parent(n)

            if suc.x == goal.x and suc.y == goal.y:
                # goal is found
                return suc

            # compute f of node
            suc.set_g(n.get_g() + h(n, suc))
            suc.set_f(suc.get_g() + h(suc, goal))

            # check if successor should be skipped
            skip = False
            all_opens = open_lst.queue
            for op in all_opens:
                if suc == op[1] and suc.get_f() >= op[0]:
                    skip = True

            for cl in close_lst:
                if suc == cl and cl.get_f() < suc.get_f():
                    skip = True

            if not skip:
                open_lst.put((suc.get_f(), suc))

        close_lst.append(n)
    raise Exception("No Possible A* path from start to goal")

def fastest_route(start, goal, world, occupied):
    """
    start and goal are in matrix i,j notation not world x,y notation
    similarly, occupied cells are also in matrix i,j notation
    """
    for occ in occupied:
        world[occ[0]][occ[1]] = 1
    world_map = Map(world)
    s_n = Node(start[1], start[0]) # x = j, y = i
    g_n = Node(goal[1], goal[0])

    goal_node = a_star_search(s_n, g_n, world_map)
    cur = deepcopy(goal_node)
    backwards_path = [] # goal to start
    while cur is not None:
        backwards_path.append(cur)
        cur = cur.parent
    path = backwards_path[::-1] # start to goal
    xs = [elem.get_x() for elem in path]
    ys = [elem.get_y() for elem in path]

    coordinates = list(zip(xs, ys)) # x, y coordinates in world
    return coordinates
