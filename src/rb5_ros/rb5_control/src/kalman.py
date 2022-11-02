import tf, rospy
import tf.transformations as t
import numpy as np
import os
import scipy


class KalmanFilter:
    def __init__(self, s0, sigma0, dt, calibration):
        r = 0.025 # radius of wheel
        lx = 0.055 # half of distance between front wheel and back wheel
        ly = 0.07 # half of distance between left wheel and right wheel
        denom = lx + ly

        self.calibration = 150.0
        os.environ["TWIST_CALIBRATION"] = "150.0"
        self.tl = tf.TransformListner()
        self.tb = tf.TransformBroadcaster()

        # KF variables
        self.dt = dt
        self.s = s0.reshape(3, 1)
        self.sigma = sigma0
        self.F = np.eye(len(s0))
        self.G = np.array([
            [1, 1, 1, 1],
            [-1, 1, 1, -1],
            [-1/denom, 1/denom, -1/denom, 1/denom]
        ]) * (self.dt * r/4)
        self.Q = 1e-3 * np.eye(len(s0)) # system noise

        self.landmarks_seen = []
        self.landmark2idx = {}

    def get_state(self):
        return self.s

    def calculate_Gu(self, twist_cmd)):
        desired_twist = self.calibration * np.array([[twist_cmd.linear.x], [twist_cmd.linear.y], [twist_cmd.angular.z]])
        # calculate the jacobian matrix
        jacobian_matrix = np.array([[1, -1, -(self.lx + self.ly)],
                                     [1, 1, (self.lx + self.ly)],
                                     [1, 1, -(self.lx + self.ly)],
                                     [1, -1, (self.lx + self.ly)]]) / self.r
        # calculate the desired wheel velocity
        result = np.dot(jacobian_matrix, desired_twist)
        wheel_speeds = result[:, 0].reshape(4, 1)

        d_xr = np.matmul(self.G, wheel_speeds) # 3 x 1
        Gu = np.zeros(self.s.shape)
        Gu[:3] = d_xr
        return Gu

    def predict(self, twist_cmd):
        s_pred = np.matmul(self.F, self.s) + self.calculate_Gu(twist_cmd)
        sigma_pred = np.matmul(self.F, (np.matmul(self.sigma, self.F.T))) + self.Q
        return s_pred, sigma_pred

    def update_state(s_pred, sigma_pred):

        frames_avail = tl.getFrameStrings()
        landmarks_avail = [name for name in frames_avail if "marker" in name]

        # add unseen landmarks to state
        for lm in landmarks_avail:
            try:
                idx = self.landmark2idx[lm]
            except KeyError: # have not seen it before
                self.landmark2idx[lm] = len() # if none seen, i=0 is assigned
                self.landmarks_seen.append(lm) # increment length
                idx = self.landmark2idx[lm]

                # add to state and covariance
                (trans, rot) = self.tl.lookupTransform("map", lm, rospy.Time())
                rot = t.euler_from_quaternion(rot)
                tag_pose = np.hstack([trans, rot]).reshape(-1, 1)

                s_pred = np.vstack([s_pred, tag_pose])
                tag_cov = np.eye(tag_pose.shape[0])
                sigma_pred = scipy.linalg.block_diag(sigma_pred, tag_cov)
                self.F = np.eye(sigma_pred.shape[0])
                self.Q = 1e-3 * np.eye(sigma_pred.shape[0])

        # create H matrix
        H = np.zeros((6*len(landmarks_avail), s_pred.shape[0]))
        H[:, :3] = np.array([])
        # create z -- landmarks must be in same order as added to state
        for lm in self.landmarks_seen:
            start_idx = self.landmark2idx[lm] + 3 # first 3 are for xr
            end_idx = start_idx + 6 # landmark pose is 6d

            H_row = np.ones()
