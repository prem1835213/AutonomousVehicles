import rospy
import tf
import numpy as np
from tf import transformations as t
from tf import LookupException
from april_detection.msg import AprilTagDetectionArray
import geometry_msgs.msg as gm

class PoseEstimator:

	def __init__(self):
		self.tl = tf.TransformListener()
		self.markers = [0, 1, 2, 3, 4, 5, 7, 8, 9]

	def tf_callback(self, transformations):
		for id in self.markers:
			try:
				(trans, rot) = self.tl.lookupTransform("camera", "marker_{}".format(id), rospy.Time(0))
				tag_T_cam = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
				cam_T_tag = t.inverse_matrix(tag_T_cam) # transformation from camera frame to tag frame
				print(cam_T_tag)
				print(id)
			except LookupException:
				pass


if __name__ == "__main__":
	rospy.init_node("pose_estimator")
	listener = PoseEstimator()
	rospy.Subscriber("/tf", gm.TransformStamped, listener.tf_callback, queue_size=10)
	rospy.spin()
