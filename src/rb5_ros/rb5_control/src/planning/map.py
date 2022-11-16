from .node import Node

class Map:
    def __init__(self, world_map):
        self.map = world_map # 2D numpy boolean array, 1 means occupied

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
        # compare node x to  columns, node y to rows
        valid_neighbors = [node for node in possible_neighbors if node.get_x() >= 0 and node.get_x() < self.map.shape[1]
                              and node.get_y() >= 0 and node.get_y() < self.map.shape[0]
                              and self.map[node.get_y(), node.get_x()] == 0
                          ]
        return valid_neighbors
