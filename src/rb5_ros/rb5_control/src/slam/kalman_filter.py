import tf
import rospy
import numpy as np
import tf.transformations as t
import os
import math
from scipy.linalg import block_diag
from april_detection.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from landmark import Landmark


class KalmanFilter:

	def __init__(self, init_state):
		self.s = init_state.reshape(-1, 1)
		assert self.s.shape[0] == 3 and self.s.shape[1] == 1
		self.sigma = 1e-3 * np.eye(3)

		# self.R = 1e-3 * np.eye(3) # measurement uncertainity, calibrate from pose samples
		# R measured based upon avg of 3 stationary potions' cov matrices of poses
		self.R = 1e-3 * np.array([
			[1.0, 0.0, 0.0],
			[0.0, 1.0, 0.0],
			[0.0, 1.0, 10.0]
		])

		self.Q = 1e-3 * np.eye(3) # system uncertainity
		self.init_tag_uncertainty = 1e-3 * np.eye(3)

		self.landmarks_seen = []
		self.landmark2idx = {}

		self.tl = tf.TransformListener()
		self.tb = tf.TransformBroadcaster()

	def _update_state(self, known_status):
        # known_status is list(tuple(bool, Landmark, index))

        # create H
		H_right = []
        known_indexes = [tup[2] if tup[0] == True for tup in known_status] # k
		for i in range(len(self.landmarks_seen)):
			if i in known_indexes:
				rowmat = np.zeros((3, 3*len(self.landmarks_seen)))
				rowmat[:, i*3:i*3+3] = np.eye(3)
				H_right.append(rowmat)
		H_right = np.vstack(H_right)
        assert H_right.shape[0] == 3 * len(known_indexes) # check if all known-in-view are in H
        assert H_right.shape[1] == self.s.shape[0] - 3


		H_left = [-1 * np.eye(3)] * len(known_indexes)
		H_left = np.vstack(H_left) # 3k x 3
		H = np.hstack([H_left, H_right])
		assert H.shape[0] == 3 * len(known_indexes) and H.shape[1] == self.s.shape[0]

		# rotate H to robot frame
		theta_r = self.s[2][0]
		w_R_r = np.array([
			[np.cos(theta_r), -np.sin(theta_r), 0.0],
			[np.sin(theta_r), np.cos(theta_r), 0.0],
			[0.0, 0.0, 1.0]
		])
		r_R_w = [w_R_r.T] * len(known_indexes)
		r_R_w = block_diag(*r_R_w)

		H = np.matmul(r_R_w, H)

        all_indexes = [tup[2] for tup in known_status] # helper to retrieve landmark obj

        # create z in robot frame
		z = []
		for i in range(len(self.landmarks_seen)):
			if i in known_indexes:
                index_in_known_status = all_indexes.index(i)
                lm = known_status[index_in_known_status]
                z += [lm.get_x(), lm.get_y(), lm.get_theta()]

		z = np.array(z).reshape(-1, 1) # 3k x 1
		assert z.shape[0] == 3 * len(known_indexes)

		# create S
		R = [self.R] * len(known_indexes)
		R = block_diag(*R)
		S = np.matmul(H, np.matmul(self.sigma, H.T)) + R # 3k x 3k
		assert S.shape[0] == 3 * len(known_indexes) and S.shape[1] == 3 * len(known_indexes)

		# create K
		K = np.matmul(self.sigma, np.matmul(H.T, 1e-10 + np.linalg.inv(S))) # d x 3k
		assert K.shape[0] == self.s.shape[0] and K.shape[1] == 3 * len(known_indexes)

		# update
		error = z - np.dot(H, self.s)
		error[::-3][::-1] = (error[::-3][::-1] + np.pi) % (2*np.pi) - np.pi # put angles back in -pi to pi
		self.s = self.s + np.matmul(K, error)
		self.sigma = np.matmul(np.eye(self.s.shape[0]) - np.matmul(K, H), self.sigma)

	def _expand_state(self, known_status):
		for tup in known_status:
            known, lm, index = tup
            if not known:
                self.landmarks_seen.append(lm)
                m_P_tag = np.array([lm.get_x(), lm.get_y(), lm.get_theta()]).reshape(-1, 1)
                self.s = np.vstack([self.s, m_P_tag])
                self.sigma = block_diag(self.sigma, self.init_tag_uncertainty)
                self.Q = block_diag(self.Q, np.zeros((3, 3)))

	def predict(self, update_value):

		update_value = update_value.reshape(-1,1)
		assert update_value.shape[0] == 3 and update_value.shape[1] == 1
		self.s[:3] = self.s[:3] + update_value # F & G are identity matrix
		self.sigma = self.sigma + self.Q # F is identity matrix


	def update(self, detections):
        """
        Let internal data structure stored not just be index - landmark id anymore
        Let internal data structure stored be mapping of index to Landmark Class
        Let Landmark class handle distance closeness checking
        """
        # create Landmark Objects
        landmarks_in_view = []
        for det in detections:
            r_trans = np.array([det.pose.position.x, det.pose.position.y, det.pose.position.z])
            r_rot = np.array([det.pose.orientation.x, det.pose.orientation.y, det.pose.orientation.z, det.pose.orientation.w])

            m_P_t = self.get_object_pose(r_trans, r_rot)
            m_trans = t.translation_from_matrix(m_P_t)
            m_theta_t = math.atan2(m_P_t[1][2], m_P_t[0][2])

            lm = Landmark(id=det.id, x=m_trans[0], y=m_trans[1], theta=m_theta_t)
            landmarks_in_view.append(lm)

        # identify known and unknown landmarks
        known_status = [(True, lm, self.landmarks_seen.index(lm)) if lm in self.landmarks_seen else (False, lm, 0.1) for lm in landmarks_in_view]

        num_knowns = sum([elem[0] for elem in known_status])
        if num_knowns > 0:
			# update based upon tags seen currently in view
			self._update_state(known_status)

		if num_knowns < len(landmarks_in_view):
			# expand state to include new landmarks
			self._expand_state(known_status)

	def get_robot_pose(self):
		"""Returns robot pose in map frame"""
		trans = np.zeros(3)
		trans[:2] = self.s[:2].squeeze()
		q = t.quaternion_from_euler(0, 0, self.s[2][0])

		m_T_r = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(q))
		return m_T_r

	def get_object_pose(self, obj_trans, obj_q):
		"""Returns an objects pose in map frame given pose in robot frame"""
		m_T_r = self.get_robot_pose()
		r_P_o = t.concatenate_matrices(t.translation_matrix(obj_trans), t.quaternion_matrix(obj_q))
		m_P_o = np.matmul(m_T_r, r_P_o)
		return m_P_o

	def get_state(self):
		return self.s
