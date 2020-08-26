"""
Camera publisher

Published: /rpi_cam
"""

import picamera
import cv2
import cv_bridge
import time

import rospy
from sensor_msgs import Image

picam = picamera.PiCamera()
raw = picamera.PiRGBArray()
bridge = cv_bridge.CvBridge()


def publish_camera_msgs():
	#Create publisher
	cam_pub = rospy.Publisher('/rpi_cam', Image, queue_size=10)
	rospy.init_node('imu_streamer', anonymous=True)

	# Cycle publisher
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# Get image as opencv array
		camera.capture(rawCapture, format="bgr")
		image = rawCapture.array

		# Fill Image message
		msg = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")

		# Publish Image
		cam_pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		time.sleep(0.1)
		camera.resolution = (600, 800)

		publish_camera_msgs()

	except rospy ROSInterruptException as e:
		print('camera_streamer: ', e)