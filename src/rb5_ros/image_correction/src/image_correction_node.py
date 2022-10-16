import rospy
import yaml
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters

class ImageCorrectorNode:
    def __init__(self):
        self.pub = rospy.Publisher("/camera_0/image_corrected", Image, queue_size=10)
        with open("/root/ost.yaml", "r") as f:
            self.calib_data = yaml.safe_load(f)

        def img_callback(self, img_msg):
            bridge = CvBridge()
            img = bridge.imgmsg_to_cv2(img_msg)
            h, w = img.shape[:2]

            cam_mat = np.array(self.calib_data["camera_matrix"]["data"]).reshape(3, 3)
            dist = np.array(self.calib_data['distortion_coefficients']['data'])
            img_corrected = img.copy()
            img_corrected = cv2.undistort(img, cam_mat, dist)

            img_msg_out = bridge.cv2_to_imgmsg(img_corrected, "bgr8")
            self.pub.publish(img_msg_out)

if __name__ == "__main__":
    image_corrector_node = ImageCorrectorNode()
    rospy.init_node("image_corrector")
    rospy.Subscriber("/camera_0/image_raw", Image, image_corrector_node.img_callback, queue_size=1)

    rospy.spin()
