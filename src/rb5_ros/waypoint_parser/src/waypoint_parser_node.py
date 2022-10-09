#!/usr/bin/env python
import os
import sys
import math
import time
import rospy
from sensor_msgs.msg import Joy


class Robot:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def set_theta(self, theta):
        self.theta = theta

class WaypointParserNode:
    def __init__(self, waypoints):
        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        self.waypoints = waypoints
        self.robot = Robot(0, 0, 0)


        # constants which define 1 unit in each direction
        # (ie. 1 unit forward/back, 1 unit full rotation, 1 unit slide)

        self.FORWARD_UNIT = 5.4
        self.SLIDE_UNIT = 0 # slide unsupported at this moment due to imperfect calibration
        self.ROTATE_FULL = 5.4

    def set_waypoints(self, waypoints):
        self.waypoints = waypoints

    def get_duration_straight(self, amount):
        # x, y deltas are in units, so they are proportions
        return self.FORWARD_UNIT * amount

    def get_duration_rotate(self, amount):
        return self.ROTATE_FULL * amount / (2 * math.pi)

    def waypoints_to_commands(self):
        commands = []
        for wp in self.waypoints:
            x2, y2, theta2 = wp

            # turn to 0
            if self.robot.theta >= 0:
                command = {"direction": "rotate cw", "duration": self.get_duration_rotate(abs(self.robot.theta))}
            else:
                command = {"direction": "rotate ccw", "duration": self.get_duration_rotate(abs(self.robot.theta))}
            commands.append(command)
            self.robot.set_theta(0)

            # move to x2
            d_x = x2 - self.robot.x
            if d_x >= 0:
                command = {"direction": "forward", "duration": self.get_duration_straight(abs(d_x))}
            else:
                command = {"direction": "backward", "duration": self.get_duration_straight(abs(d_x))}
            commands.append(command)
            self.robot.set_x(x2)

            # turn to pi/2 if necessary, then move to y2
            d_y = y2 - self.robot.y
            if d_y != 0:
                commands.append({"direction": "rotate ccw", "duration": 0.25*self.ROTATE_FULL})
                self.robot.set_theta(math.pi / 2)
            if d_y >= 0:
                command = {"direction": "forward", "duration": self.get_duration_straight(abs(d_y))}
            else:
                command = {"direction": "backward", "duration": self.get_duration_straight(abs(d_y))}
            commands.append(command)
            self.robot.set_y(y2)

            # turn theta2
            theta2 = theta2 % (2 * math.pi)
            d_theta = theta2 - self.robot.theta
            if d_theta >= 0:
                command = {"direction": "rotate ccw", "duration": self.get_duration_rotate(abs(d_theta))}
            else:
                command = {"direction": "rotate cw", "duration": self.get_duration_rotate(abs(d_theta))}
            commands.append(command)
            self.robot.set_theta(theta2)

        return commands

    def command_to_joy_msg(self, command):
        direction = command["direction"]
        joy_msg = Joy()
        joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

        if direction == "forward":
            joy_msg.axes[1] = 1.0
        elif direction == "backward":
            joy_msg.axes[1] = -1.0
        elif direction == "rotate ccw":
            joy_msg.axes[2] = 1.0
        elif direction == "rotate cw":
            joy_msg.axes[2] = -1.0
        elif direction == "slide left":
            joy_msg.axes[0] = 1.0
        elif direction == "slide right":
            joy_msg.axes[0] = -1.0
        else:
            print("Unknown direction given as command")

        return joy_msg

    def run(self):
        commands = self.waypoints_to_commands()
        for command in commands:

            joy_msg = self.command_to_joy_msg(command)
            self.pub_joy.publish(joy_msg)

            time.sleep(command["duration"])

            stop_joy_msg = Joy()
            stop_joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            stop_joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
            self.pub_joy.publish(stop_joy_msg)


if __name__ == "__main__":
    with open("/waypoints.txt", "r") as f:
        lines = f.readlines()
    waypoints = []
    for line in lines:
        line = line.replace("\n", "")
        line = line.split(",")
        line = [float(elem) for elem in line]
        waypoints.append(line)

    waypoint_parser_node = WaypointParserNode(waypoints=waypoints)
    rospy.init_node("waypoint_parser")
    waypoint_parser_node.run()


