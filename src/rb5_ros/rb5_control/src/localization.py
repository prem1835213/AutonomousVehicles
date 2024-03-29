import numpy as np
import time
import math
import tf
import rospy
import tf.transformations as t
from april_detection.msg import AprilTagDetectionArray

def error(s1, s2):
    result = s1 - s2
    result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
    return np.linalg.norm(result)

def create_state(trans, q):
    rot = t.quaternion_matrix(q)
    theta = math.atan2(rot[1][2], rot[0][2])
    state = np.array([trans[0], trans[1], theta])
    return state

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
        # keep duplicate tags on opposite sides of map
		self.markers = {
			0: [WorldTag(1, [2.7125, 0.625, 0.0])],
			9: [WorldTag(1, [2.7125, 1.25, 0.0]), WorldTag(2, [1.875, 2.7125, 0.0]), WorldTag(2, [0.625, 2.7125, 0.0]), WorldTag(3, [-0.2125, 0.625, 0.0]), WorldTag(4, [1.875, -0.2125, 0.0])],
			1: [WorldTag(1, [2.7125, 1.875, 0.0])],
			2: [WorldTag(2, [1.25, 2.7125, 0.0])],
			8: [WorldTag(3, [-0.2125, 1.875, 0.0])],
			5: [WorldTag(3, [-0.2125, 1.25, 0.0])],
			7: [WorldTag(4, [0.625, -0.2125, 0.0])],
			4: [WorldTag(4, [1.25, -0.2125, 0.0])]
        }
		self.markers_used = [0, 1, 2, 3, 4, 5, 7, 8, 9]

	def calculate_world_P_cam(self, tag_id, k, tl):
		now = rospy.Time()
		tl.waitForTransform("camera", "marker_{}".format(tag_id), now, rospy.Duration(0.1))
		trans, rot = tl.lookupTransform("camera", "marker_{}".format(tag_id), now)
		cam_T_tag = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
		tag_T_cam = t.inverse_matrix(cam_T_tag)
		world_P_cam = np.matmul(self.markers[tag_id][k].w_T_tag, tag_T_cam)
		return world_P_cam

	def estimate_pose(self, detections, estimated_state, tl, tb):
		ids_found = [det.id for det in detections]
		ids_found = [id for id in ids_found if id in self.markers_used]
		for tag_id in ids_found:
			if len(self.markers[tag_id]) == 1:
				try:
					world_P_cam = self.calculate_world_P_cam(tag_id, k=0, tl=tl)
					trans = t.translation_from_matrix(world_P_cam)
					rot = t.quaternion_from_matrix(world_P_cam)
					tb.sendTransform(trans, rot, rospy.Time.now(), "camera", "world")
					print("Published world transform based on tag {}".format(tag_id))
				except:
					continue
			else:
				print("Ambiguity for tag {}, multiple known locations".format(tag_id))

				best_error = np.inf
				true_w_P_cam = None
				best_k = 0
				for k in range(len(self.markers[tag_id])):
					world_P_cam = self.calculate_world_P_cam(tag_id, k=k, tl=tl)
					r_state = create_state(t.translation_from_matrix(world_P_cam), t.quaternion_from_matrix(world_P_cam))
					err = error(r_state, estimated_state)
					if err < best_error:
						true_world_P_cam = world_P_cam
						best_k = k
						best_error = err
				true_cam_trans = t.translation_from_matrix(true_world_P_cam)
				true_cam_q = t.quaternion_from_matrix(true_world_P_cam)
				tb.sendTransform(true_cam_trans, true_cam_q, rospy.Time(), "camera", "world")
				print("Published camera pose for tag {id} based on location {k}".format(id=tag_id, k=best_k))
