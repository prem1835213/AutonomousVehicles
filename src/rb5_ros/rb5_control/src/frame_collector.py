import rospy
import tf
import time
from april_detection.msg import AprilTagDetectionArray

class FrameCollector:

	def __init__(self):
		self.ids_avail = []
		self.last_collected = time.time()
	
	def collect(self, april_array):
		self.ids_avail = [det.id for det in april_array.detections]
		self.last_collected = time.time()
	
	def query(self):
		if time.time() - self.last_collected < 0.1:
			return self.ids_avail
		else:
			return []
