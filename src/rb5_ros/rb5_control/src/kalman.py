import tf, rospy
import tf.transformations as t
import numpy as np
import os
import math
from scipy.linalg import block_diag


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
        # self.G = np.array([
        #     [1, 1, 1, 1],
        #     [-1, 1, 1, -1],
        #     [-1/denom, 1/denom, -1/denom, 1/denom]
        # ]) * (self.dt * r/4)
        self.Q = 1e-3 * np.eye(len(s0)) # system noise

        # measurement uncertainty
        self.R = np.array([
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ])

        self.landmarks_seen = []
        self.landmark2idx = {}

    def get_state(self):
        return self.s

    def calculate_Gu(self, update_value)):
        # desired_twist = self.calibration * np.array([[twist_cmd.linear.x], [twist_cmd.linear.y], [twist_cmd.angular.z]])
        # # calculate the jacobian matrix
        # jacobian_matrix = np.array([[1, -1, -(self.lx + self.ly)],
        #                              [1, 1, (self.lx + self.ly)],
        #                              [1, 1, -(self.lx + self.ly)],
        #                              [1, -1, (self.lx + self.ly)]]) / self.r
        # # calculate the desired wheel velocity
        # result = np.dot(jacobian_matrix, desired_twist)
        # wheel_speeds = result[:, 0].reshape(4, 1)

        d_xr = np.matmul(self.G, update_value) # 3 x 1
        Gu = np.zeros(self.s.shape)
        Gu[:3] = d_xr
        return Gu

    def _wrap(self, angle):
        """Puts an angle in range 0-2pi"""
        tmp = np.arctan2(np.sin(angle), np.cos(angle))
        return float(np.where(tmp<0 , 2*np.pi+tmp, tmp))

    def _unwrap(self, angle):
        """Puts an angle in -pi to pi"""
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        return angle

    def predict(self, update_value):
        # given that update_value is in world frame and if PID is calibrated properly, it will be equal to deltas, is there a need for Gu?
        update_value = update_value.squeeze().reshape(3, 1)
        # s_pred = np.matmul(self.F, self.s) + self.calculate_Gu(update_value)
        s_pred = self.s.copy()
        s_pred[:3] = s_pred[:3] + update_value # same update from open loop PID
        sigma_pred = np.matmul(self.F, (np.matmul(self.sigma, self.F.T))) + self.Q
        return s_pred, sigma_pred

    def _update(self, s_pred, sigma_predstill_in_view):
        """Updates state based upon measurement error wrt landmarks STILL IN VIEW"""

        H_rows = []
        for i in range(len(still_in_view)):
            marker_name = still_in_view[i]
            # add 3 rows for tx, ty, t_theta
            row1 = np.zeros(s_pred.shape[0]) # dim(s)
            row2 = np.zeros(s_pred.shape[0]) # dim(s)
            row3 = np.zeros(s_pred.shape[0]) # dim(s)

            row1[0] = -1 # subtracting xr
            row2[1] = -1 # subtracting yr
            row3[2] = -1 # subtracting thetar

            tag_x_idx, tag_y_idx, tag_theta_idx = self.landmark2idx[marker_name]

            row1[tag_x_idx] = 1 # adding t_xm
            row2[tag_y_idx] = 1 # adding t_ym
            row3[tag_theta_idx] = 1 # adding t_thetam

            H_rows.append(row1)
            H_rows.append(row2)
            H_rows.append(row3)
        H = np.vstack(H_rows) # 3*k x d where k = # seen tags still in view

        w_P_tags = np.matmul(H, s_pred) # 3k x 1 -- tag pose wrt world frame

        # ROTATE w_P_tags to be in Robot frame
        theta_r = s_pred[2][0]
        w_R_r = np.array([
            [np.cos(theta_r), -np.sin(theta_r), 0],
            [np.sin(theta_r), np.cos(theta_r), 0],
            [0, 0, 1]
        ])
        r_R_w = w_R_r.T # 3x3
        lst = [r_R_w] * len(still_in_view)
        r_R_w = block_diag(*lst) # 3k x 3k

        H = np.matmul(r_R_w, H) # 3k x d

        # construct z vector -- xytheta of tags in robot frame
        z = []
        for i in range(len(still_in_view)):
            marker_name = still_in_view[i]

            # need to publish static transform to go from camera to robot
            try:
                now = rospy.Time()
                self.tl.waitForTransform("robot", marker_name, now, rospy.Duration(0.001))
                (trans, rot) = self.tl.lookupTransform("robot", marker_name, now)
            except tf.LookupException as e:
                print("TF LOOKUP EXCEPTION")
                print(e)

            matrix = quaternion_matrix(rot)
            ttheta = math.atan2(matrix[1][2], matrix[0][2])
            ttheta = self._wrap(ttheta) # convert to 0-2pi
            z += [trans[0], trans[1], ttheta]
        z = np.array(z).reshape(-1, 1) # 3k x 1

        # R = measurement uncertainty
        R = self.R.copy()
        lst = [R] * len(still_in_view)
        R = block_diag(*lst) # 3k x 3k

        S = np.matmul(H, np.matmul(sigma_pred, H.T)) + R # 3k x 3k
        K = np.matmul(sigma_pred, np.matmul(H.T, np.linalg.inv(S)))

        # update
        Hs = np.matmul(H, s_pred)
        Hs[::3][::-1] = self._unwrap(Hs[::3][::-1]) # convert back to -pi to pi

        self.s = self.s + np.matmul(K, (z - Hs))
        self.sigma = np.matmul(np.eye(self.s.shape[0]) - np.matmul(K, H), sigma_pred)

    def update(s_pred, sigma_pred, still_in_view):
        # first calculate error with already seen landmarks STILL in view
        # update s and sigma
        # add new landmarks to state, expand sigma

        frames_avail = self.tl.getFrameStrings()
        landmarks_avail = set([name for name in frames_avail if "marker" in name])
        still_in_view = list(landmarks_avail.intersection(set(self.landmarks_seen)))
        new_landmarks = list(landmarks_avail - set(self.landmarks_seen))

        if len(still_in_view) > 0:
            # does Kalman Update equations for self.s and self.sigma
            self._update(s_pred, sigma_pred, still_in_view)
        else:
            # when no measurements are available we cannot update - uncertainty in position increases
            self.s = s_pred
            self.sigma = sigma_pred

        xr, yr, thetar = self.s[:3]
        trans = np.array([xr, yr, 0])
        q = t.quaternion_from_euler(0, 0, thetar, axes='sxyz')
        self.tb.sendTransform(trans, q, rospy.Time(), "robot", "map") # use this transform for estimating tag in map

        # expand s, sigma, Q for new landmarks
        for i in range(len(new_landmarks)):
            marker_name = new_landmarks[i]

            self.landmarks_seen.append(marker_name)
            tag_x_idx = len(self.s)
            tag_y_idx = tag_x_idx + 1
            tag_theta_idx = tag_y_idx + 1
            self.landmark2idx[marker_name] = [tag_x_idx, tag_y_idx, tag_theta_idx]

            # calculate tag pose wrt to map
            # need to publish static transform to go from camera to robot
            try:
                now = rospy.Time()
                self.tl.waitForTransform("map", marker_name, now, rospy.Duration(0.001))
                (trans, rot) = self.tl.lookupTransform("map", marker_name, now)
                [t_xm, t_ym, t_zm] = trans
                rot = t.quaternion_matrix(rot)
                t_thetam = math.atan2(matrix[1][2], matrix[0][2])

                self.s = np.vstack([self.s, np.array([t_xm, t_ym, t_thetam])]) # d+3 x 1
                tag_cov = 1e-3 * np.eye(3)
                self.sigma = np.block_diag(self.sigma, tag_cov)
                self.Q = 1e-3 * np.eye(self.sigma.shape[0])
            except tf.LookupException as e:
                print("TF LOOKUP EXCEPTION")
                print(e)
