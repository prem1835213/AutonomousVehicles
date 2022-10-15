#!/usr/bin/env python
import os
import yaml
import sys
import math
import time
import rospy
from sensor_msgs.msg import CameraInfo


class CameraInfoNode:
    def __init__(self, calib_data):
        self.pub = rospy.Publisher("/camera_0/camera_info", CameraInfo, queue_size=1)
        self.camera_info_msg = None

    def set_camera_info_msg(self):
        now = rospy.Time.now()

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.stamp = now
        self.camera_info_msg.width = calib_data['image_width']
        self.camera_info_msg.height = calib_data["image_height"]
        self.camera_info_msg.K = calib_data["camera_matrix"]["data"]
        self.camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        self.camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        self.camera_info_msg.P = calib_data["projection_matrix"]["data"]
        self.camera_info_msg.distortion_model = calib_data["distortion_model"]


    def run(self):
        while True:
            self.set_camera_info_msg()
            print(self.camera_info_msg.header.stamp)
            self.pub.publish(self.camera_info_msg)


if __name__ == "__main__":

    with open("/root/ost.yaml", "r") as f:
        calib_data = yaml.safe_load(f)

    camera_info_node = CameraInfoNode(calib_data)

    rospy.init_node("camera_info")
    camera_info_node.run()


