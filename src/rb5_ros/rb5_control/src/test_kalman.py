from kalman_filter import KalmanFilter
import os
import time
import numpy as np
import rospy
import tf
from tf import transformations as t
from april_detection.msg import AprilTagDetectionArray
from frame_collector import FrameCollector

if __name__ == "__main__":
	rospy.init_node("kalman_test")
	fc = FrameCollector()
	april_sub = rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, fc.collect, queue_size=1)
	tl = tf.TransformListener()

	sigma0 = 1e-3 * np.eye(3)
	current_state = np.array([0.0, 0.0, 0.0])
	
	kf = KalmanFilter(current_state)
	
	u = np.array([0, 0, 0])

	kf.predict(u)
	kf.update(fc.query())
	current_state = kf.get_state()

	print("State after First Movement: ")
	print(current_state) # should be 1, 0, 0, tx, ty, ttheta

	print("Sleeping before next movement")
	time.sleep(5)
	u = np.array([0, 0, 0])
	kf.predict(u)
	kf.update(fc.query())

	print("State after Second Movement: ")
	current_state = kf.get_state()
	print(current_state)

