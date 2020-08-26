"""
Camera Controller
"""

import picamera
import picamera.array
import cv2
import time


class Camera:

	def __init__(self):
		self.cam = picamera.PiCamera()
		self.raw = picamera.array.PiRGBArray(self.cam)

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
		self.cam.capture(self.raw, format='bgr')
		image = self.raw.array

		return image

	"""
	Save a frame from the camera
	"""
	def save_frame(self):
		image = self.get_cv_frame()
		cv2.imwrite('test.png', image)
		print('Wrote image to current directory')

	"""
	View camera video stream until 'q'
	"""
	def show_stream(self):
		for frame in self.cam.capture_continuous(self.raw, format="bgr", use_video_port=True):
			image = frame.array

			cv2.imshow("Picamera", image)
			key = cv2.waitKey(1) & 0xFF
			raw.truncate(0)

			if key == ord("q"):
				break


if __name__ == '__main__':
	# Test camera stream
	print("Initializing camera")
	c = Camera()
	print("Camera setup complete")
	c.save_frame()
