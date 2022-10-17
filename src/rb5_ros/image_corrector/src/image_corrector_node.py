#!/usr/bin/env python
import rospy
import yaml
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageCorrectorNode:
    def __init__(self):
        self.pub = rospy.Publisher("/camera_0/image_corrected", Image, queue_size=10)
        with open("/root/ost.yaml", "r") as f:
            self.calib_data = yaml.safe_load(f)

    def img_callback(self, img_msg):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg)
        h, w = img.shape[:2]

        C = self.calib_data["camera_matrix"]["data"]
        C = np.array(C).reshape(3, 3)
        D = np.array(self.calib_data["distortion_coefficients"]["data"])

        new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(C, D, (w, h), 1, (w, h))

        corrected_img = img.copy()
        corrected_img = cv2.undistort(img, C, D, None, new_camera_mtx)

        x, y, w, h = roi
        corrected_img = corrected_img[y:y+h, x:x+w]

        corrected_img_msg = bridge.cv2_to_imgmsg(corrected_img, "rgb8")

        self.pub.publish(corrected_img_msg)


if __name__ == "__main__":
    image_corrector_node = ImageCorrectorNode()
    rospy.init_node("image_corrector")
    rospy.Subscriber("/camera_0/image_raw", Image, image_corrector_node.img_callback, queue_size=10)

    rospy.spin()

