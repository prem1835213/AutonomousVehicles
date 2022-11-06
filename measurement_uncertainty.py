import rospy
import tf
import time
import numpy as np
import tf.transformations as t
import math

from april_detection.msg import AprilTagDetectionArray

class MeasurementUncertainty:
	def __init__(self):
		self.xs = []
		self.ys = []
		self.thetas = []

		self.r_T_c = np.array([
			[0, 0, 1, 0],
			[-1, 0, 0, 0],
			[0, -1, 0, 0],
			[0, 0, 0, 1]
		])
	
	def collect_measurement(self, april_array):
		print(len(april_array.detections))
		det = april_array.detections[0]

		pose = det.pose
		[x, y, z] = [pose.position.x, pose.position.y, pose.position.z]
		q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
		
		trans = np.array([x, y, z])
		rot = np.array(q)

		c_P_t = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
		r_P_t = np.matmul(self.r_T_c, c_P_t)

		tag_trans = t.translation_from_matrix(r_P_t)
		tag_theta = math.atan2(r_P_t[1][2], r_P_t[0][2])

		self.xs.append(tag_trans[0])
		self.ys.append(tag_trans[1])
		self.thetas.append(tag_theta)

	def compute_covariance(self):
		X = np.vstack([np.array(self.xs), np.array(self.ys), np.array(self.thetas)]) # 3 x len(self.xs)
		cov = np.cov(X)

		return cov



if __name__ == "__main__":
	rospy.init_node("measurement_uncertainty")
	
	mu = MeasurementUncertainty()
	april_sub = rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, mu.collect_measurement, queue_size=1)

	time.sleep(15)

	print("Collected {} measurements: ".format(len(mu.xs)))
	cov = mu.compute_covariance()
	print(cov)
