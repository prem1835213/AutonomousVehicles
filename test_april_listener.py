import rospy
import tf
import numpy as np
from tf import transformations as t
from tf import LookupException
from april_detection.msg import AprilTagDetectionArray
import geometry_msgs.msg as gm

#								tag2
				####################
				#	+y							 #	
				# ^								 # tag1
# tag3	# |								 #  
				#  --> +x          #
				####################
#				      tag4


# constant rotation matrices for tags on 4 walls given diagram above
TAG1_ROTATION = np.array([[0, 0, 1],
													[-1, 0, 0],
													[0, -1, 0]
													])

TAG2_ROTATION = np.array([
												 [1, 0, 0],
												 [0, 0, 1],
												 [0, -1, 0]])


TAG3_ROTATION = np.array([
												 [0, 0, -1],
												 [1, 0, 0],
												 [0, -1, 0]])

TAG4_ROTATION = np.array([
												 [-1, 0, 0],
												 [0, 0, -1],
												 [0, -1, 0]])

class WorldTag:
	"""
	This class contains logic which generates the transformation matrix w_T_tag
	
	This is equivalent to the pose of tag in world frame
	It is also equivalent to the transformation matrix which transforms poses in tag frame to a pose in world frame.
	"""
	def __init__(self, tag_type, translation):

		if tag_type == 1:
			self.rotation = TAG1_ROTATION
		elif tag_type == 2:
			self.rotation = TAG2_ROTATION
		elif tag_type == 3:
			self.rotation = TAG3_ROTATION
		elif tag_type == 4:
			self.rotation = TAG4_ROTATION
		else:
			raise Exception("Invalid tag type provided")

		self.translation = t.translation_matrix(np.array(translation))
		
		# making rotation matrix 4x4 [[R, [0, 0, 0]], [t, 1]]
		self.rotation = np.hstack([self.rotation, np.array([0, 0, 0]).reshape(3, 1)]) # 3 x 4
		self.rotation = np.vstack([self.rotation, np.array([0, 0, 0, 1]).reshape(1, 4)]) # 4 x 4

		self.w_T_tag = t.concatenate_matrices(self.translation, self.rotation)


class PoseEstimator:

	def __init__(self):
		self.tl = tf.TransformListener()
		self.markers = [0, 1, 2, 3, 4, 5, 7, 8, 9]

		self.tag_9 = WorldTag(tag_type=1, translation=[1, 0, 0])

	def tf_callback(self, transformations):
		for id in self.markers:
			try:
				(trans, rot) = self.tl.lookupTransform("camera", "marker_{}".format(id), rospy.Time(0))
				cam_P_tag = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
				tag_P_cam = t.inverse_matrix(cam_P_tag) # transformation from camera frame to tag frame
				
				if id == 9:
					w_P_cam = np.matmul(self.tag_9.w_T_tag, tag_P_cam)
					
					print("Camera pose in World View:")
					print(w_P_cam)

					print("Camera translation in World View:")
					print(t.translation_from_matrix(w_P_cam))

					print("Camera Rotation in World View:")
					print(t.rotation_from_matrix(w_P_cam))
			
			except LookupException:
				pass


if __name__ == "__main__":
	rospy.init_node("pose_estimator")
	listener = PoseEstimator()
	rospy.Subscriber("/tf", gm.TransformStamped, listener.tf_callback, queue_size=10)
	rospy.spin()
