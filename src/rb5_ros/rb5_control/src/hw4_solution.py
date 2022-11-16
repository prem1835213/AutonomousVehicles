#!/usr/bin/env python
import argparse
import tf
from tf import transformations as t
import sys
import rospy
from geometry_msgs.msg import Twist
from april_detection.msg import AprilTagDetectionArray
import numpy as np
from planning import fastest_route, safest_route
from planning import ROBOT_WIDTH, ROBOT_HEIGHT
from localization import PoseEstimator, create_state
import math
import time

"""
The class of the pid controller.
"""
class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.05
        self.maximumValue = 1.5

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value.
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0]
    twist_msg.linear.y = desired_twist[1]
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)

class Robot:
	def __init__(self, init_state):
		self.tl = tf.TransformListener()
		self.state = init_state
		self.detected_at = time.time()
        self.pe = PoseEstimator()

	def store_detections(self, apriltag_array):
		self.detections = apriltag_array.detections
		if len(self.detections) > 0:
			self.detected_at = time.time()

	def update_state(self, update_value):
		if time.time() - self.detected_at < 0.05 and len(self.detections) > 0:
			self.pe.estimate_pose(self.detections)
			trans, q = self.tl.lookupTransform("world", "camera", rospy.Time())
			self.state = create_state(trans, q)
		else:
			self.state = self.state + update_value
			self.state[2] = (self.state[2] + np.pi) % (2 * np.pi) - np.pi

	def get_state(self):
		return self.state

def convert_cells_to_waypoints(coords):
    """coords are in x, y notation on world map, need to convert to waypoints"""
    waypoints = []
    for elem in coords:
        # x is correct, but y is positive in opposite direction
        w_x, w_y = elem
        w_y = world.shape[0] - 1 - w_y
        waypoint_x = w_x*(ROBOT_WIDTH) + (ROBOT_WIDTH/2)
        waypoint_y = w_y *(ROBOT_HEIGHT) + (ROBOT_HEIGHT/2)
        waypoints.append([waypoint_x, waypoint_y])
    return np.array(waypoints)

def compress_waypoints(wayps):
    """turns sequence of waypoints into smaller list by considering continuity in slope"""
    denom = (wayps[1][0] - wayps[0][0])
    if denom == 0:
        prev_slope = np.inf
    else:
        prev_slope = (wayps[1][1] - wayps[0][1]) / denom
    condensed = [wayps[0]]

    for k in range(1, len(wayps)-1):
        wp = wayps[k]
        denom = (wayps[k+1][0] - wp[0])
        if denom == 0:
            slope_with_next = np.inf
        else:
            slope_with_next = (wayps[k+1][1] - wp[1]) / denom

        if slope_with_next != prev_slope:
            condensed.append(wp)
        prev_slope = slope_with_next
    condensed.append(wayps[-1])
    return np.array(condensed)

def theta_to_next(wps):
    """
    returns theta for each waypoint which will point to
    the next waypoint
    """
    thetas = []
    for k in range(len(wps)-1):
        dy = wps[k+1][1] - wps[k][1]
        dx = wps[k+1][0] - wps[k][0]
        theta = math.atan2(dy, dx)
        thetas.append(theta)
    return np.array(thetas)

parser = argparse.ArgumentParser(description='Fast or Safe route')
parser.add_argument('-f', '--fast', action='store_true')
parser.add_argument('-s', '--safe', action='store_true')

def state_to_transform(state):
    x, y, theta = state
    trans = np.array([x, y, 0.0])
    camera_aligned = np.array([
        [0, 0, 1],
        [-1, 0, 0],
        [0, -1, 0]
    ])
    rotation = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ]) # rotation of camera about world's z axis in CCW
    camera_axes = rotation.dot(camera_aligned)
    w_R_cam = np.vstack([rotation, np.zeros(3).reshape(1, -1)]) # 4 x 3
    w_R_cam = np.hstack([w_R_cam, np.array([0, 0, 0, 1]).reshape(-1, 1)]) # 4 x 4
    q = t.quaternion_from_matrix(w_R_cam)
    return trans, q

if __name__ == "__main__":

	args = parser.parse_args()
	if args.fast and args.safe:
		raise Exception("Cannot be both fast and safe")
	elif not args.fast and not args.safe:
		raise Exception("Must pick a mode of fast or safe")

	rospy.init_node("hw4")
	current_state = np.array([0.0, 0.0, 0.0])

	robot = Robot(init_state=current_state)
	rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, robot.store_detections, queue_size=1)
	pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

	tb = tf.TransformBroadcaster()

	world = np.zeros((12, 12))
	# next two lines are world INDEX locations of which cells the obstacles are placed at
	occupied_cells = [(5 ,5), (5, 6), (6, 5), (6, 6)]
	obstacle_centers = [(5.5, 5.5)]

	if args.fast:
		move_cells = fastest_route(start=(11, 11), goal=(0, 0), world=world, occupied=occupied_cells)
	else:
		move_cells = safest_route(start=[11, 11], goal=[0, 0], world=world, obs_centers=obstacle_centers)

	waypoint = convert_cells_to_waypoints(move_cells)
	waypoint = compress_waypoints(waypoint)
	waypoint = np.hstack([waypoint, np.zeros((waypoint.shape[0], 1))]) # add theta dimension
	waypoint[:-1, 2] = theta_to_next(waypoint[:]) # modify theta dimension to point to next waypoint
	waypoint[-1, 2] = waypoint[-2, 2] # end pointing in same direction as n-1 waypoint to prevent extra turn

	pid = PIDcontroller(0.0175, 0.001, 0.00025)

	# current_state = np.array([2.875, 0.125, np.pi/2]) # start at center of bottom right cell facing up

	update_value = np.array([1.0, 0.0, 0.0])
	print("Sleeping for 3 seconds")
	time.sleep(3)

	est_trans, est_q = state_to_transform(current_state + update_value)
	tb.sendTransform(est_trans, est_q, rospy.Time(), "estimated_camera", "world")
	robot.update_state(update_value)
	current_state = robot.get_state()
	print("State after moving:")
	print(current_state)


	# for wp in waypoint:
	# 	print("move to way point", wp)
	# 	pid.setTarget(wp)
    #
	# 	update_value = pid.update(current_state)
	# 	pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
	# 	time.sleep(0.05)
    #
    #     est_trans, est_q = state_to_transform(current_state + update_value)
    #     tb.sendTransform(est_trans, est_q, rospy.Time(), "estimated_camera", "world")
	# 	if abs(time.time() - robot.changed_at) < 0.05:
	# 		current_state = robot.current_state
	# 	else:
	# 		current_state += update_value
    #         current_state[2] = (current_state[2] + np.pi) % (2 * np.pi) - np.pi
    #
	# 	while(np.linalg.norm(pid.getError(current_state, wp)) > 0.1):
	# 		update_value = pid.update(current_state)
	# 		pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
	# 		time.sleep(0.05)
    #
    #         est_trans, est_q = state_to_transform(current_state + update_value)
    #         tb.sendTransform(est_trans, est_q, rospy.Time(), "estimated_camera", "world")
	# 		if abs(time.time() - robot.changed_at) < 0.05:
	# 			current_state = robot.current_state
	# 		else:
	# 			current_state += update_value
    #             current_state[2] = (current_state[2] + np.pi) % (2 * np.pi) - np.pi
    #
    # # stop the car and exit
	# pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
