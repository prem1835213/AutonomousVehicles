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
