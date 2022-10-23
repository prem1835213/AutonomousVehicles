import rospy
import numpy as np
import tf
from tf import transformations as t
from tf import LookupException
import geometry_msgs.msg as gm


class Robot:

	def __init__(self, init_pose):
		self.pose = np.array(init_pose)
		self.tl = tf.TransformListener()
		self.perceived_pose = None
		self.r = rospy.Rate(10)
	



if __name__ == "__main__":
	waypoints = [
		[0.0, 0.0, 0.0],
		[0.75, 0.0, 0.0],
		[0.75, 1.5, np.pi],
		[0.0, 0.0, 0.0]
	]
	current_pose = np.array([0, 0, 0])
	for wp in waypoints:
		print("move to waypoint", wp)

		while np.linalg.norm(current_pose, np.array(wp)) > EPSILON:
			
			
			
