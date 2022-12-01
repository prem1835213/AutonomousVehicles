import numpy as np
class Landmark:
    def __init__(self, id, x, y, theta, tolerance=0.15):
        """
        Landmark is defined by Tag ID, x, y, theta

        :param id: ID of the April Tag which defines this Landmark
        :param x: Landmark x coordinate in map frame
        :param y: Landmark y coordinate in map frame
        :param y: Landmark theta coordinate in map frame
        :param tolerance: within what distance should another landmark be considered the same
        """
        self.id = id
        self.x = x
        self.y = y
        self.theta = theta
        self.tolerance = tolerance

    def get_id(self):
        return self.id

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_theta(self):
        return self.theta

    def __str__(self):
        print("ID: {} x: {} y: {} theta{}".format(self.id, self.x, self.y, self.theta))

    def __eq__(self, other):
        """
        Method to override == comparison for class equality checking

        :param other: Another object of type Landmark
        :return: True if the Landmark ``other`` can be considered the same Landmark
        """
        d = np.sqrt((self.x - other.get_x())**2 + (self.y - other.get_y())**2)
        return self.id == other.get_id() and d < self.tolerance
