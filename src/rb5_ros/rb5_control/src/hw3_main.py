#!/usr/bin/env python
import os
import tf
from tf import transformations as t
import sys
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from kalman_filter import KalmanFilter
from frame_collector import FrameCollector
from april_detection.msg import AprilTagDetectionArray

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
		self.maximumValue = 0.1

	def setTarget(self, targetx, targety, targetw):
		"""set the target pose."""
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
	def __init__(self):
		self.tl = tf.TransformListener()

	def estimate_pose(self):
		print("Estimating Pose..")
		self.tl.waitForTransform("camera", "world", rospy.Time(0), rospy.Duration(1))
		(trans, rot) = self.tl.lookupTransform("world", "camera", rospy.Time(0))
		rot = t.quaternion_matrix(rot)

		# z axis is +x in robot frame, rotate camera z axis by world rotation to get robot heading
		rotated_z = np.matmul(rot, np.array([0, 0, 1, 1]).reshape(4, 1)).squeeze()
		theta = np.arctan(rotated_z[1] / rotated_z[0])

		x = rotated_z[0]
		y = rotated_z[1]
		if x <= 0 and y >= 0: # quadrant 2, arctan will produce negative theta_
			theta = np.pi + theta
		elif x <= 0 and y <= 0: # quadrant 3, arctan will produce positive theta_
			theta = np.pi + theta
		elif x >=0 and y <=0: # quadrant 4, arctan will produce negative theta_
			theta = 2*np.pi + theta
		pose = np.array([trans[0], trans[1], theta])
		# roll, pitch, yaw = t.euler_from_quaternion(rot)
		# pose = np.array([trans[0], trans[1], yaw])
		# print(pose)
		return pose

if __name__ == "__main__":

	import time
	rospy.init_node("hw3")
	robot = Robot()
	pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

	waypoint = np.array([[0.0, 0.0, 0.0], [0.75, 0.0, 0.0], [0.75, 1.5, np.pi], [0.0, 0.0, 0.0]])
	waypoint = waypoint[:, :]

	# drive in square -- only consider errors for specific directions
	# start at [6, 2, pi/2] drive in square CCW
	waypoint = [
	    [0.0, 0.0, 0.0],
	    [1.0, 0.0, 0.0],
	    [1.0, 0.0, np.pi/2],
	    [1.0, 1.0, np.pi/2],
	    [1.0, 1.0, np.pi],
	    [0.0, 1.0, np.pi],
	    [0.0, 1.0, -np.pi/2],
	    [0.0, 0.0, -np.pi/2],
	    [0.0, 0.0, 0.0]
	]
	waypoint = waypoint[:3]
	# waypoint = [[0.0, 0.0, 0.0], [0.75, 0.0, 0.0], [0.0, 0.0, 0.0]]
	# zeroing-out error terms might not be a good idea because it will prevent proper correction

	# init pid controller
	pid = PIDcontroller(0.0175, 0.001, 0.00025)

	# init current state
	current_state = np.array([0.0,0.0,0.0])
	tl = tf.TransformListener()
	kf = KalmanFilter(current_state)
	fc = FrameCollector()
	april_sub = rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, fc.collect, queue_size=1)

	for wp in waypoint:
		print("move to way point", wp)
		pid.setTarget(wp)

		# calculate the current twist
		update_value = pid.update(current_state)
		twist_msg = genTwistMsg(coord(update_value, current_state))
		pub_twist.publish(twist_msg)
		time.sleep(pid.timestep)

		# conduct estimation of state via KalmanFilter
		kf.predict(update_value)
		kf.update(fc.query())
		current_state = kf.get_state()[:3, 0] # x, y, theta
		
		i = 0
		while(np.linalg.norm(pid.getError(current_state, wp)) > 0.1) and i < 100:
			if i % 25 == 0:
				print(current_state)
			i += 1

			# calculate the current twist
			update_value = pid.update(current_state)
			twist_msg = genTwistMsg(coord(update_value, current_state))
			pub_twist.publish(twist_msg)
			time.sleep(pid.timestep)

			# update the current state
			kf.predict(update_value)
			kf.update(fc.query())
			current_state = kf.get_state()[:3, 0] # x, y, theta

	print(kf.get_state())
	print("Landmarks seen: ", kf.landmarks_seen)
	# stop the car and exit
	pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
