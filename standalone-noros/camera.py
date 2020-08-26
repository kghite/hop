"""
Camera Controller
"""

import picamera
import cv2
import time


class Camera:

	def __init__(self):
		self.cam = picamera.PiCamera()
		self.raw = picamera.PiRGBArray()

		# Camera setup
		self.cam.resolution = (640, 480)
		self.cam.framerate = 10
		time.sleep(0.5)

	"""
	Get single frame from camera
	Return:
		* image - numpy array (640, 480)
	"""
	def get_cv_frame(self):
		self.cam.capture(raw, format='bgr')
		image = raw.array

		return image

	"""
	View camera video stream until 'q'
	"""
	def show_stream(self):
		for frame in self.cam.capture_continuous(raw, format="bgr", use_video_port=True):
			image = frame.array

			cv2.imshow("Picamera", image)
			key = cv2.waitKey(1) & 0xFF
			raw.truncate(0)

			if key == ord("q"):
				break


if __name__ == '__main__':
	# Test camera stream
	c = Camera()
	c.show_stream()
