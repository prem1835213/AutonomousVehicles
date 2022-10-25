import time
import rospy
import numpy as np
import tf
from tf import transformations as t
from tf import LookupException
from sensor_msgs.msg import Joy


class Robot:
	def __init__(self):
		self.tl = tf.TransformListener()
	
	def estimate_pose(self):
		print("Estimating Pose..")
		self.tl.waitForTransform("camera", "world", rospy.Time(0), rospy.Duration(1))
		(trans, q) = self.tl.lookupTransform("world", "camera", rospy.Time(0))
		rot = t.quaternion_matrix(q)
		
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
		theta = t.euler_from_quaternion(q)[2] + np.pi/2
		pose = np.array([trans[0], trans[1], theta])
		# roll, pitch, yaw = t.euler_from_quaternion(rot)
		# pose = np.array([trans[0], trans[1], yaw])
		# print(pose)
		return pose

class Controller:
	def __init__(self):
		self.pub = rospy.Publisher("/joy", Joy, queue_size=10)
	
	def execute_command(self, command, frac=1.0):
		command_str, duration = command
		duration = duration * frac
		joy_msg = Joy()
		joy_msg.axes = 8 * [0.0]
		joy_msg.buttons = 8 * [0]

		if command_str == "forward":
			joy_msg.axes[1] = 1.0
		elif command_str == "backward":
			joy_msg.axes[1] = -1.0
		elif command_str == "rotate ccw":
			joy_msg.axes[2] = 1.0
		elif command_str == "rotate cw":
			joy_msg.axes[2] = -1.0
		else:
			raise Exception("unknown command given")
		
		self.pub.publish(joy_msg)
		time.sleep(duration)

	def stop_motors(self):
		joy_msg = Joy()
		joy_msg.axes = 8 * [0.0]
		joy_msg.buttons = 8 * [0]
		self.pub.publish(joy_msg)

class Planner:
	def __init__(self):
		self.FULL_CIRCLE = 5.4 # duration for a full rotation
		self.UNIT = 5.4 # duration for 1 meter
	
	def turn_to_point(self, pose1, pose2):
		x1, y1, theta1 = pose1
		x2, y2, theta2 = pose2
		if x2 == 0:
			x2 = 1e-10
		# turn to x2, y2
		theta_ = np.arctan(y2/x2)
		if x2 <= 0 and y2 >= 0: # quadrant 2, arctan will produce negative theta_
			theta_ = np.pi + theta_
		elif x2 <= 0 and y2 <= 0: # quadrant 3, arctan will produce positive theta_
			theta_ = np.pi + theta_
		elif x2 >=0 and y2 <=0: # quadrant 4, arctan will produce negative theta_
			theta_ = 2*np.pi + theta_

		d_theta_ = theta_ - theta1
		if abs(d_theta_) <= 0.05:
			return ["rotate ccw", 0]
		elif d_theta_ >= 0:
			return ["rotate ccw", self.FULL_CIRCLE * (d_theta_/(2*np.pi))], theta_
		else:
			return ["rotate cw", self.FULL_CIRCLE * (abs(d_theta_)/(2*np.pi))], theta_
	
	def move_to_point(self, pose1, pose2):
		# move to x2, y2
		x1, y1, theta1 = pose1
		x2, y2, theta2 = pose2
		dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
		return ["forward", dist * self.UNIT]

	def adjust_heading(self, heading1, heading2):
		d_theta = heading2 - heading1
		if abs(d_theta) <= 0.05:
			return ["rotate ccw", 0]
		elif d_theta >= 0:
			return ["rotate ccw", self.FULL_CIRCLE * d_theta/(2*np.pi)]
		else:
			return ["rotate cw", self.FULL_CIRCLE * abs(d_theta)/(2*np.pi)]

	def plan_movement(self, pose1, pose2):
		x1, y1, theta1 = pose1
		x2, y2, theta2 = pose2
		
		# turn to x2, y2
		turn_to_point_command, theta_ = self.turn_to_point(pose1, pose2)

		# move to x2, y2
		move_to_point_command = self.move_to_point(pose1, pose2)

		# turn to theta2
		# currently facing theta_
		final_heading_command = self.adjust_heading(theta_, theta2)

		commands = [turn_to_point_command, move_to_point_command, final_heading_command]
		return commands



if __name__ == "__main__":
	rospy.init_node("closed_loop_controller")
	controller = Controller()
	planner = Planner()
	robot = Robot()

	waypoints = [[0, 0, 0], [0.75, 0, 0], [0.75, 1.5, np.pi], [0, 0, 0]]
	waypoints = np.array(waypoints)[:4]
	current_pose = np.array([0, 0, 0])

	while True:
		print(robot.estimate_pose())
		time.sleep(1)


	for i in range(len(waypoints)):
		if i == 0:
			continue
		wp = waypoints[i]
		print(wp)
		plan = planner.plan_movement(current_pose, wp)
		print(plan)
		current_pose = robot.estimate_pose()
		print("Current Pose before turn-to: ", current_pose)
		controller.execute_command(plan[0], frac=1.0) # execute turn-to-point
		controller.stop_motors()
		# get feedback
		current_pose = robot.estimate_pose()
		print("Pose after turn-to: ", current_pose)
		move_to_command = planner.move_to_point(current_pose, wp)
		controller.execute_command(move_to_command, frac=0.5) # move 1/2 distance
		controller.stop_motors()
		# get feedback
		current_pose = robot.estimate_pose()
		print("Pose after move-to: ", current_pose)
		new_plan = planner.plan_movement(current_pose, wp)
		# execute new plan completely
		for k in range(len(new_plan)):
			c = new_plan[k]
			controller.execute_command(c, frac=1.0)
			controller.stop_motors()
			current_pose = robot.estimate_pose()
			print("Pose after New plan k={} :".format(k))
			print(current_pose)
		current_pose = robot.estimate_pose()
		print("Pose after all waypoint-plan: ", current_pose)
		localization_error = np.linalg.norm(current_pose[:2] - wp[:2])
		print("Localization Error for Waypoint ", wp)
		print(localization_error)
	
	print("Done with waypoints.")

