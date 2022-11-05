from ..kalman import KalmanFilter
import os
import numpy as np
import rospy
import tf
from tf import transformations as t

rospy.init_node("kalman_test")

current_state = np.array([0.0, 0.0, 0.0])
sigma0 = 1e-3 * np.eye(3)

kf = KalmanFilter(curent_state, sigma0)

u = np.array([1, 0, 0])
