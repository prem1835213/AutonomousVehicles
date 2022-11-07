import tf
import rospy
import numpy as np
import tf.transformations as t
import os
import math
from scipy.linalg import block_diag
from april_detection.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist


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

	def _update_state(self, found_ids):
		# create H
		H_right = []
		for i in range(len(self.landmarks_seen)):
			if self.landmarks_seen[i] in found_ids:
				rowmat = np.zeros((3, 3*len(self.landmarks_seen)))
				rowmat[:, i*3:i*3+3] = np.eye(3)
				H_right.append(rowmat)
		H_right = np.vstack(H_right)
		
		found_already_seen = list(set(found_ids).intersection(set(self.landmarks_seen)))

		H_left = [-1 * np.eye(3)] * len(found_already_seen)
		H_left = np.vstack(H_left) # 3k x 3
		H = np.hstack([H_left, H_right])
		assert H.shape[0] == 3*len(found_already_seen) and H.shape[1] == self.s.shape[0]
		
		# rotate H to robot frame
		theta_r = self.s[2][0]
		w_R_r = np.array([
			[np.cos(theta_r), -np.sin(theta_r), 0.0],
			[np.sin(theta_r), np.cos(theta_r), 0.0],
			[0.0, 0.0, 1.0]
		])
		r_R_w = [w_R_r.T] * len(found_already_seen)
		r_R_w = block_diag(*r_R_w)
		
		H = np.matmul(r_R_w, H)
		
		# create z in robot frame
		z = []
		for id in self.landmarks_seen:
			marker_name = "marker_{}".format(id)
			if id in found_ids:
				try:
					now = rospy.Time()
					self.tl.waitForTransform("robot", marker_name, now, rospy.Duration(0.1))
					(trans, rot) = self.tl.lookupTransform("robot", marker_name, now)
					rot = t.quaternion_matrix(rot)
					r_theta_t = math.atan2(rot[1][2], rot[0][2])
					z += [trans[0], trans[1], r_theta_t]
				except tf.LookupException:
					print("TF LOOKUP EXCEPTION tag in robot")
					print(e)
		z = np.array(z).reshape(-1, 1) # 3k x 1
		assert z.shape[0] == 3*len(found_already_seen)
		
		# create S
		R = [self.R] * len(found_already_seen)
		R = block_diag(*R)
		S = np.matmul(H, np.matmul(self.sigma, H.T)) + R # 3k x 3k
		assert S.shape[0] == 3*len(found_already_seen) and S.shape[1] == 3*len(found_already_seen)
		
		# create K
		K = np.matmul(self.sigma, np.matmul(H.T, 1e-10 + np.linalg.inv(S))) # d x 3k
		assert K.shape[0] == self.s.shape[0] and K.shape[1] == 3*len(found_already_seen)
		
		# update
		error = z - np.dot(H, self.s)
		error[::-3][::-1] = (error[::-3][::-1] + np.pi) % (2*np.pi) - np.pi # put angles back in -pi to pi
		self.s = self.s + np.matmul(K, error)
		self.sigma = np.matmul(np.eye(self.s.shape[0]) - np.matmul(K, H), self.sigma)

	def _expand_state(self, unknown_ids):
		for id in unknown_ids:
			marker_name = "marker_{}".format(id)
			self.landmarks_seen.append(id)
			# utilize pose of robot in map frame published by predict step
			now = rospy.Time()
			self.tl.waitForTransform("robot", marker_name, now, rospy.Duration(0.5))
			(trans, rot) = self.tl.lookupTransform("robot", marker_name, now)
			
			m_P_t = self.get_object_pose(trans, rot)
			trans = t.translation_from_matrix(m_P_t)
			
			# calculate tag pose in map frame
			m_theta_t = math.atan2(m_P_t[1][2], m_P_t[0][2])
			m_P_tag = np.array([trans[0], trans[1], m_theta_t]).reshape(-1, 1)
			self.s = np.vstack([self.s, m_P_tag])
			self.sigma = block_diag(self.sigma, self.init_tag_uncertainty)
			self.Q = block_diag(self.Q, np.zeros((3, 3))) # movement of robot does not affect landmark uncertainty

	def predict(self, update_value):

		update_value = update_value.reshape(-1,1)
		assert update_value.shape[0] == 3 and update_value.shape[1] == 1
		self.s[:3] = self.s[:3] + update_value # F & G are identity matrix
		self.sigma = self.sigma + self.Q # F is identity matrix


	def update(self, ids_found):
		known_ids = list(set(self.landmarks_seen).intersection(set(ids_found)))
		unknown_ids = list(set(ids_found).difference(set(self.landmarks_seen)))
		
		if len(known_ids) > 0:
			# update based upon tags seen currently in view
			self._update_state(ids_found)
		
		if len(unknown_ids) > 0:
			# expand state to include new landmarks
			self._expand_state(unknown_ids)
		
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
