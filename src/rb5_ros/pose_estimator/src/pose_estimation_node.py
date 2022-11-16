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

def error(s1, s2):
    result = s1 - s2
    result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
    return np.linalg.norm(result)

def create_state(trans, q):
    rot = t.quaternion_matrix(q)
    theta = math.atan2(rot[1][2], rot[0][2])
    state = np.array([trans[0], trans[1], theta])
    return state

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
		self.markers_used = [0, 4]

        # keep duplicate tags on opposite sides of map
        self.markers = {
            0: [WorldTag(tag_type=4), WorldTag(tag_type=2)],
            1: [WorldTag(tag_type=4), WorldTag(tag_type=2)],
            2: [WorldTag(tag_type=3), WorldTag(tag_type=1)],
            3: [WorldTag(tag_type=3), WorldTag(tag_type=1)],
            4: [WorldTag(tag_type=3)],
            5: [WorldTag(tag_type=4)],
            7: [WorldTag(tag_type=1)],
            8: [WorldTag(tag_type=2)]
        }
        self.markers_used = [0, 1, 2, 3, 4, 5, 7, 8]

    def calculate_world_P_cam(self, tag_id, k):
        trans, rot = self.tl.lookupTransform("camera", "marker_{}".format(tag_id), rospy.Time(0))
        cam_T_tag = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
        tag_T_cam = t.inverse_matrix(cam_T_tag)
        world_P_cam = np.matmul(self.markers[tag_id][k].w_T_tag, tag_T_cam)
        return world_P_cam

	def april_callback(self, array_msg):
		detections = array_msg.detections
		ids_found = [det.id for det in detections]
		ids_found = [id for id in ids_found if id in self.markers_used]
		for tag_id in ids_found:
            if len(self.markers[tag_id]) == 1:
                try:
                    world_P_cam = self.calculate_world_P_cam(tag_id, k=0)
					trans = t.translation_from_matrix(world_P_cam)
					rot = t.quaternion_from_matrix(world_P_cam)
					self.tb.sendTransform(trans, rot, rospy.Time.now(), "camera", "world")
					print("Published world transform based on tag {}".format(tag_id))
                except:
                    continue
            else:
                print("Ambiguity for tag {}, multiple known locations".format(tag_id))
                self.tl.waitForTransform("world", "estimated_camera", rospy.Time(), 0.01)
                est_trans, est_q = self.tl.lookupTransform("world", "estimated_camera", rospy.Time(0))
                est_state = create_state(est_trans, est_q)

                best_error = np.inf
                true_w_P_cam = None
                best_k = 0
                for k in range(len(self.markers[tag_id])):
                    world_P_cam = self.calculate_world_P_cam(tag_id, k=k)
                    r_state = create_state(t.translation_from_matrix(world_P_cam), t.quaternion_from_matrix(world_P_cam))
                    err = error(r_state, est_state)
                    if err < best_error:
                        true_world_P_cam = world_P_cam
                        best_k = k
                        best_error = err
                true_cam_trans = t.translation_from_matrix(true_world_P_cam)
                true_cam_q = t.quaternion_from_matrix(true_world_P_cam)
                self.tb.sendTransform(true_cam_trans, true_cam_q, rospy.Time(), "camera", "world")
                print("Published camera pose for tag {id} based on location {k}".format(id=tag_id, k=best_k))

if __name__ == "__main__":
	print("Initializing Pose Estimator")
	rospy.init_node("pose_estimator")
	pose_node = PoseEstimator()
	rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, pose_node.april_callback, queue_size=1)
	rospy.spin()
