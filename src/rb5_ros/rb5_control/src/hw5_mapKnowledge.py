#!/usr/bin/env python
import argparse
import tf
from tf import transformations as t
import sys
import rospy
from geometry_msgs.msg import Twist
from april_detection.msg import AprilTagDetectionArray
import numpy as np
from coverage import bfs_coverage, ROBOT_WIDTH, ROBOT_HEIGHT, Node, Map
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
        self.timestep = 0.1
        self.maximumValue = 0.5

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
		self.tb = tf.TransformBroadcaster()
		self.state = init_state
		self.detected_at = time.time()
		self.pe = PoseEstimator()

	def store_detections(self, apriltag_array):
		self.detections = apriltag_array.detections
		if len(self.detections) > 0:
			print("Updating detections")
			self.detected_at = time.time()

	def update_state(self, update_value):
		try:
			array_msg = rospy.wait_for_message("/apriltag_detection_array", AprilTagDetectionArray, timeout=1.0)
			detections = array_msg.detections
			estimated_state = self.state + update_value
			estimated_state[2] = (estimated_state[2] + np.pi) % (2*np.pi) - np.pi
			if len(detections) > 0:
				self.pe.estimate_pose(detections, estimated_state, self.tl, self.tb)
				now = rospy.Time()
				self.tl.waitForTransform("world", "camera", now, rospy.Duration(0.1))
				trans, q = self.tl.lookupTransform("world", "camera", now)
				self.state = create_state(trans, q)
			else:
				print("Detections empty")
				self.state = estimated_state
		except rospy.ROSException:
			print("No Tag detection message received")
			self.state = self.state + update_value
			self.state[2] = (self.state[2] + np.pi) % (2*np.pi) - np.pi


	def get_state(self):
		return self.state

def coordinate_to_waypoints(coords, world_matrix):
    """
    turns matrix i=y, j=x values into robot waypoints
    """
    # world is 12 x 12
    waypoints = []
    for elem in coords:
        # x is correct, but y is positive in opposite direction
        w_x, w_y = elem
        w_y = world_matrix.shape[0] - 1 - w_y
        waypoint_x = w_x*(ROBOT_WIDTH) + (ROBOT_WIDTH/2)
        waypoint_y = w_y *(ROBOT_HEIGHT) + (ROBOT_HEIGHT/2)
        waypoints.append([waypoint_x, waypoint_y])
    return waypoints

def compress_waypoints(wayps):
    """
    turns sequence of waypoints into smaller list
    by considering continuity in slope
    """
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
        else:
            if slope_with_next == np.inf and prev_slope == np.inf:
                prev_num = wp[1] - wayps[k-1][1]
                cur_num = wayps[k+1][1] - wp[1]
                if (prev_num < 0 and cur_num > 0) or (prev_num > 0 and cur_num < 0):
                    condensed.append(wp)
            elif slope_with_next == 0 and prev_slope == 0:
                # Just because slope is 0 (could be -dx or +dx)
                prev_denom = wp[0] - wayps[k-1][0]
                if (denom < 0 and prev_denom > 0) or (denom > 0 and prev_denom < 0):
                    condensed.append(wp)

        prev_slope = slope_with_next
    condensed.append(wayps[-1])
    return np.array(condensed)

def theta_from_prev(wps):
    """
    returns theta for each waypoint (except first) which will in direction of travel from previous to next waypoint
    """
    thetas = []
    for k in range(1, len(wps)):
        dy = wps[k][1] - wps[k-1][1]
        dx = wps[k][0] - wps[k-1][0]
        theta = math.atan2(dy, dx)
        thetas.append(theta)
    return np.array(thetas)

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

	rospy.init_node("hw5_mapKnowledge")

    INIT_THETA = np.pi/2
    current_state = np.array([2.5-0.125, 0.125, INIT_THETA])

	robot = Robot(init_state=current_state)
	pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

	tb = tf.TransformBroadcaster()

	world_matrix = np.zeros((10, 10)) # map size is 2.5m x 2.5m

    world_map = Map(world)
    move_cells = bfs_coverage(start=Node(9, 9), mapp=world_map)
    xs = [node.get_x() for node in move_cells]
    ys = [node.get_y() for node in move_cells]
    coordinates = np.array(list(zip(xs, ys)))

    waypoints = coordinate_to_waypoints(coordinates, world_matrix)
	condensed_wps = compress_waypoints(waypoints)
    init_thetas = INIT_THETA * np.ones((condensed_wps.shape[0], 1))
    theta_wps = np.hstack([condensed_wps, init_thetas])
    theta_wps[1:, 2] = theta_from_prev(theta_wps[:]) # direction from previous to next

    # add virtual waypoints to discourage twisting motion
    trajectory = []
    for i in range(len(theta_wps)-1):
        trajectory.append(theta_wps[i])
        virtual_wp = theta_wps[i].copy()
        virtual_wp[2] = theta_wps[i+1][2]
        trajectory.append(virtual_wp)
    trajectory.append(theta_wps[-1])
    trajectory = np.array(trajectory)

	pid = PIDcontroller(0.0175, 0.001, 0.00025)

    print("Desired Trajectory:")
    print(trajectory)

	poses = [current_state]
	for wp in trajectory:
		print("move to way point", wp)
		pid.setTarget(wp)

		update_value = pid.update(current_state)
		pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
		time.sleep(0.1)

		robot.update_state(update_value)
		current_state = robot.get_state()
		poses.append(current_state)
		i = 0
		while(np.linalg.norm(pid.getError(current_state, wp)) > 0.1) and i < 10:
			i += 1
			update_value = pid.update(current_state)
			pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
			time.sleep(0.1)

            pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0]))) # stop when updating
            robot.update_state(update_value)
			current_state = robot.get_state()
			poses.append(current_state)
			# print(current_state)

    # stop the car and exit
	pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
	# print(current_state)
	poses.append(current_state)
