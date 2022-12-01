import numpy as np
import matplotlib.pyplot as plt
import math
from PIL import Image
import cv2

from copy import deepcopy

ROBOT_WIDTH = 0.25
ROBOT_HEIGHT = 0.25

class Node:
    def __init__(self, x, y):
        # numpy array is indexed by arr[y, x]
        self.x = x
        self.y = y
        self.parent = None
        self.f = 0
        self.g = 0

    def __str__(self):
        return "x: {} y: {}".format(self.x, self.y)

    def set_parent(self, n):
        self.parent = n

    def get_parent(self):
        return self.parent

    def set_f(self, f):
        self.f = f

    def get_f(self):
        return self.f

    def set_g(self, g):
        self.g = g

    def get_g(self):
        return self.g

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def __eq__(self, n):
        return self.x == n.get_x() and self.y == n.get_y()

class Map:
    def __init__(self, world_map):
        self.map = world_map # 2D numpy boolean array, 1 means occupied
        self.visited = []

    def visit(self, nodes):
        self.visited += nodes

    def get_visited(self):
        return self.visited

    def get_neighbors(self, n):
        """Returns unoccupied neighbor cells in the map for a given node"""
        x = n.get_x()
        y = n.get_y()
        possible_neighbors = [
            Node(x, y - 1),
            Node(x, y + 1),
            Node(x - 1, y),
            Node(x + 1, y),
            Node(x - 1, y - 1),
            Node(x - 1, y + 1),
            Node(x + 1, y - 1),
            Node(x + 1, y + 1),
        ]
        valid_neighbors = [node for node in possible_neighbors if node.get_x() >= 0 and node.get_x() < self.map.shape[1]
                              and node.get_y() >= 0 and node.get_y() < self.map.shape[0]
                              and self.map[node.get_y(), node.get_x()] == 0
                          ]
        return valid_neighbors

    def get_unvisited_neighbors(self, n):
        x = n.get_x()
        y = n.get_y()
        possible_neighbors = [
            Node(x, y - 1),
            Node(x, y + 1),
            Node(x - 1, y),
            Node(x + 1, y),
            Node(x - 1, y - 1),
            Node(x - 1, y + 1),
            Node(x + 1, y - 1),
            Node(x + 1, y + 1),
        ]
        valid_neighbors = [node for node in possible_neighbors if node.get_x() >= 0 and node.get_x() < self.map.shape[1]
                              and node.get_y() >= 0 and node.get_y() < self.map.shape[0]
                              and self.map[node.get_y(), node.get_x()] == 0
                          ]
        return [ne for ne in valid_neighbors if ne not in self.visited]
