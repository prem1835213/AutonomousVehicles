import math
import numpy as np
import os


class Robot:
    def __init__(self):
        self.x = None
        self.y = None
        self.theta = None

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def set_theta(self, theta):
        self.theta = theta

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_theta(self):
        return self.theta

    def control(self, x2, y2, theta2):
        """
        Returns a series of commands to go from x1, y1, theta1 to x2, y2, theta2
        """
        # input is given in (-x, y, theta)
        x2 *= -1

        # Turn to 0
        commands = []
        if self.theta > 0:
            commands.append(["rotate", self.theta]) # self.theta because cw is positive voltage
        elif self.theta < 0:
            commands.append(["rotate", -self.theta]) # -self.theta because ccw is negative voltage

        # Move in x-direction
        commands.append(["straight", x2 - self.x]) # x2 - x because x2 has been converted to positive, and "e" is positive voltage

        # Move in y-direction
        commands.append(["slide", self.y - y2]) # y - y2 because "a" would be up and is negative voltage

        # Turn to final angle
        commands.append(["rotate", -theta2]) # -theta2 because "e" is cw, and has positive voltage, so ccw is negative

        return commands





