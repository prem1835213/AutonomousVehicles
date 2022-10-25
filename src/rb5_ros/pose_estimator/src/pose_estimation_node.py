#!/usr/bin/env python
import rospy
import time
import tf
import numpy as np
from tf import LookupException
from tf import transformations as t
from april_detection.msg import AprilTagDetectionArray
import geometry_msgs.msg as gm

TYPE1_ROTATION = np.array([
	[0, 0, 1],
	[-1, 0, 0],
	[0, -1, 0]])

TYPE2_ROTATION = np.array([
	[1, 0, 0],
	[0, 0, 1],
	[0, -1, 0]])

TYPE3_ROTATION = np.array([
	[0, 0, -1],
	[1, 0, 0],
	[0, -1, 0]])

TYPE4_ROTATION = np.array([
	[-1, 0, 0],
	[0, 0, -1],
	[0, -1, 0]])

CAM_ROTATION = np.array([
	[0, 0, 1],
	[-1, 0, 0],
	[0, -1, 0]
])

class WorldTag:

	def __init__(self, tag_type, translation):
		"""
		tag_type: type 1 is on right wall, type2 is on top wall, type 3 is on left wall, type 4 is on bototm wall assuming that world frame +x points to right and +y points to top walls

		translation: list representing position of tag in world frame [x, y, z]
		"""
		if tag_type == 1:
			self.rotation = TYPE1_ROTATION
		elif tag_type == 2:
			self.rotation = TYPE2_ROTATION
		elif tag_type == 3:
			self.rotation = TYPE3_ROTATION
		elif tag_type == 4:
			self.rotation = TYPE4_ROTATION
		else:
			raise Exception("Invalid tag type provided")

		self.translation = t.translation_matrix(np.array(translation))
		# create 4 x 4 Rotation Matrix
		self.rotation = np.hstack([self.rotation, np.array([0, 0, 0]).reshape(3, 1)])
		self.rotation = np.vstack([self.rotation, np.array([0, 0, 0, 1]).reshape(1, 4)])

		self.w_T_tag = t.concatenate_matrices(self.translation, self.rotation)


class PoseEstimator:
	def __init__(self):
		self.tl = tf.TransformListener() # listens to /tf topic
		self.tb = tf.TransformBroadcaster()
		self.markers = {
			0: WorldTag(tag_type=1, translation=[1.244, 0.037, 0.432]),
			1: WorldTag(tag_type=1, translation=[1.197, 2.033, 0.399]),
			2: WorldTag(tag_type=1, translation=[1.286, 0.885, 0.4195]),
			3: WorldTag(tag_type=2, translation=[0.711, 2.13, 0.374]),
			4: WorldTag(tag_type=3, translation=[-0.23, 1.46, 0.31]),
			5: WorldTag(tag_type=3, translation=[-0.281, 0.386, 0.285]),
			7: WorldTag(tag_type=4, translation=[0.148, -1.922, 0.339])
		}
		# self.markers = {
			# 0: WorldTag(tag_type=3, translation=[0.0, 1.0, 0.092]),
			# 1: WorldTag(tag_type=1, translation=[1.0, 0.0, 0.211])
		# }
		self.markers_used = [0, 4]
		# self.markers_used = [0, 1]
		# self.robot_T_cam = np.hstack([CAM_ROTATION, np.array([0.05, 0.02, 0.171]).reshape(3, 1)])
		# self.robot_T_cam = np.vstack([self.robot_T_cam, np.array([0, 0, 0, 1]).reshape(1, 4)])

		self.r = rospy.Rate(10)
	def april_callback(self, array_msg):
		detections = array_msg.detections
		ids_found = [det.id for det in detections]
		ids_found = [id for id in ids_found if id in self.markers_used]
		translations = []
		rotations = []
		for tag_id in ids_found:
				try:
					trans, rot = self.tl.lookupTransform("camera", "marker_{}".format(tag_id), rospy.Time(0))
					cam_T_tag = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
					tag_T_cam = t.inverse_matrix(cam_T_tag)
					world_P_cam = np.matmul(self.markers[tag_id].w_T_tag, tag_T_cam)
					trans = t.translation_from_matrix(world_P_cam)
					rot = t.quaternion_from_matrix(world_P_cam)
					translations.append(trans)
					rotations.append(rot)
					self.tb.sendTransform(trans, rot, rospy.Time.now(), "camera", "world")
					print("Published world transform based on tag {}".format(tag_id))
				except LookupException:
					continue

if __name__ == "__main__":
	print("Initializing Pose Estimator")
	rospy.init_node("pose_estimator")
	pose_node = PoseEstimator()
	rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, pose_node.april_callback, queue_size=1)
	rospy.spin()
