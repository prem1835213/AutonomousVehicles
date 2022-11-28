import numpy as np
import matplotlib.pyplot as plt
import math
from PIL import Image
import cv2

from copy import deepcopy

def h(n, g):
    """
    n and g are tuple of (row, col) coordinates in world map grid
    returns euclidean distance in meters between two coordinates in map grid
    """
    y1 = n.get_y()
    x1 = n.get_x()

    y2 = g.get_y()
    x2 = g.get_x()

    d = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    return d

def plan_path(node, remaining_nodes):
    if len(remaining_nodes) == 0:
        return []
    else:
        d_opt = np.inf
        argmin = 0
        for i in range(len(remaining_nodes)):
            d = h(node, remaining_nodes[i])
            if d < d_opt:
                d_opt = d
                argmin = i

        closest = remaining_nodes.pop(argmin)
        return [closest] + plan_path(closest, remaining_nodes)

def bfs_coverage(start, mapp):

    frontier = [start]
    cur = start
    mapp.visit([cur])

    path = [cur]
    while True:
        S = [] # next level to explore
        for i in range(len(frontier)):
            node_un = mapp.get_unvisited_neighbors(frontier[i])
            S += [n for n in node_un if n not in S]

        if len(S) == 0:
            return path

        sub_path = plan_path(cur, deepcopy(S))
        path += sub_path
        mapp.visit(S)
        cur = sub_path[-1]
        frontier = S
    return path
