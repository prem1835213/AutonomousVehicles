import rospy
import tf
import time
from april_detection.msg import AprilTagDetectionArray

class FrameCollector:

	def __init__(self):
		self.detections = []
		self.last_collected = time.time()

	def collect(self, april_array):
		self.detections = april_array.detections
		self.last_collected = time.time()

	def query(self):
		if time.time() - self.last_collected < 0.1:
			return self.detections
		else:
			return []
