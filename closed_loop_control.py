import rospy
import time
import numpy as np
import tf
from tf import transformations as t
from tf import LookupException
import geometry_msgs.msg as gm
from sensor_msgs.msg import Joy

class Robot:

	def __init__(self, init_pose):
		self.pose = np.array(init_pose)
		self.tl = tf.TransformListener()
		self.perceived_pose = None
		self.r = rospy.Rate(10)

	def estimate_pose(self):
		try:
			(trans, rot) = self.tl.lookupTransform("world", "robot", rospy.Time(0))
			roll, pitch, yaw = t.euler_from_quaternion(rot)

			pose = np.array([trans.x, trans.y, yaw])
		except LookupException:
			print("No world transform published")
			return np.array([0, 0, 0]) # TODO address no tags found in view

		return pose


class Controller:
	def __init__(self):
		self.pub = rospy.Publisher("/joy", Joy, queue_size=10)

	def execute_command(self, command):
		command_str, duration = command
		joy_msg = Joy()
		joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

		if command_str == "forward":
			joy_msg.axes[1] = 1.0
		elif command_str == "backward":
			joy_msg.axes[1] = -1.0
		elif command_str == "rotate ccw":
			joy_msg.axes[2] = 1.0
		elif command_str == "rotate cw":
			joy_msg.axes[2] = -1.0
		elif command_str == "slide left":
			joy_msg.axes[0] = 1.0
		elif command_str == "slide right":
			joy_msg.axes[0] = -1.0
		else:
			raise Exception("Unknown command given")

		self.pub.publish(joy_msg)
		time.sleep(duration)

	def stop_motors(self):
		stop_joy_msg = Joy()
		stop_joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		stop_joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
		self.pub.publish(stop_joy_msg)

class Planner:
	def __init__(self):
		pass

	def generate_command_sequence(self, pose1, pose):
		x0, y0, theta0 = pose1
		xf, yf, thetaf = pose2

		cur_x, cur_y, cur_theta = [x0, y0, theta0]
		command_sequence = []
		# turn to 0
		if cur_theta >=0:
			command = ["rotate cw", 5.4 * (cur_theta / (2*np.pi))]
		else:
			command = ["rotate ccw", 5.4 * (abs(cur_theta) / (2*np.pi))]

		command_sequence.append(command)
		cur_theta = 0

		# move to x2
		d_x = xf - cur_x
		if d_x >= 0:
			command = ["forward", 5.4 * d_x]
		else:
			command = ["backward", 5.4 * d_x]

		command_sequence.append(command)
		cur_x = xf

		# turn to pi/2 if necessary then move to y2
		d_y = yf - cur_y
		if d_y != 0:
			command = ["rotate ccw", 0.25 * 5.4]
			command_sequence.append(command)
		cur_theta = np.pi/2

		if d_y >= 0:
			command = ["forward", d_y * 5.4]
		else:
			command = ["backward", abs(d_y) * 5.4]
		command_sequence.append(command)

		# turn to thetaf
		thetaf = thetaf % (2*np.pi)
		d_theta = thetaf - cur_theta
		if d_theta >= 0:
			command = ["rotate ccw", 5.4 * (d_theta / (2*np.pi))]
		else:
			command = ["rotate cw", 5.4 * (abs(d_theta) / (2*np.pi))]
		command_sequence.append(command)
		cur_theta = thetaf

		return command_sequence


	def commands_to_waypoints(self, current_position, commands):
		"""
		Returns the expected poses based upon a series of commands
		"""
		sub_waypoints = []
		cur_x, cur_y, cur_theta = current_position
		for command in commands:
			command_str, duration = command

			if command_str == "rotate cw":
				proportion = duration / 5.4
				delta_theta = 1*2*np.pi*proportion
				cur_theta = cur_theta - delta_theta
			elif command_str == "rotate ccw":
				proportion = duration / 5.4
				delta_theta = 1*2*np.pi*proportion
				cur_theta = cur_theta + delta_theta
			elif command_str == "forward" or command_str == "backward":
				c = duration / 5.4
				if command_str == "forward":
					cur_x = cur_x + c*np.cos(cur_theta)
					cur_y = cur_y + c*np.sin(cur_theta)
				else:
					cur_x = cur_x - c*np.cos(cur_theta)
					cur_y = cur_y - c*np.sin(cur_theta)
			else:
				raise Exception("Invalid control command generated")

			sub_waypoints.append(np.array([cur_x, cur_y, cur_theta]))
		return sub_waypoints


if __name__ == "__main__":
	# Repeat the process for list of waypoints
	planner = Planner()
	controller = Controller()
	robot = Robot()

	waypoints = [np.array([0.0, 0.0, 0.0]), np.array([0.75, 0.0, 0.0])]
	for i in range(len(waypoints)):
		if i == 0:
			current_position = np.array([0.0, 0.0, 0.0])
		else:
			current_position = waypoints[i-1]

		command_seq = planner.generate_command_sequence(current_position, waypoints[i]) # plan
		sub_waypoints = planner.commands_to_waypoints(current_position, command_seq) # expected steps
		control_expectations = list(zip(command_seq, sub_waypoints))

		for j in range(len(control_expectations)):
			expectation = control_expectations[j]
			command, sub_wp = expectation
			command_str, duration = command
			max_execute_duration = 2
			controller.execute_command([command_str, min(duration, max_execute_duration)])

			if duration > max_execute_duration: # case where we cut it off for pose estimation
				current_position = robot.estimate_pose()
				duration_significant = True
				while np.linalg.norm(current_position - sub_wp) > 5 and duration_significant:
					if command_str == "rotate cw" or command_str == "rotate ccw":
						# might overshoot so recalculate direction as well
						d_theta = sub_wp[2] - current_position[2]
						dur = 5.4 * abs(d_theta) / (2*np.pi)
						if d_theta >= 0:
							com = "rotate ccw"
						else:
							com = "rotate cw"
					else:
						if abs(np.pi - sub_wp[2]) < abs(0 - sub_wp[2]):
							# we are intending on moving in y direction
							delta = sub_wp[1] - current_position[1]
                        else:
							delta = sub_wp[0] - current_position[0]
						if delta >= 0:
							com = "forward"
						else:
							com = "backward"
						dur = abs(delta) * 5.4

					controller.execute_command([com, min(dur, max_execute_duration)])
					flag = dur > max_execute_duration
                    current_position = robot.estimate_pose()

            current_position = robot.estimate_pose()

		error = np.linalg.norm(current_position - waypoints[i])
    print("localization error")
