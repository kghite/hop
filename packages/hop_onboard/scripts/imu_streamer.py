"""
IMU publisher

Published: /imu
"""

import time
import board
import busio
from adafruit_lsm6ds import LSM6DS33

import rospy
from sensor_msgs import Imu

i2c = busio.I2C(board.SCL, board.SDA)
sensor = LSM6DS33(i2c)

def publish_imu_msgs():
	#Create publisher
	imu_pub = rospy.Publisher('/imu', Twist, queue_size=10)
	rospy.init_node('imu_streamer', anonymous=True)

	# Cycle publisher
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# Fill Imu msg
		msg = Imu()
		msg.linear_acceleration = sensor.acceleration
		msg.angular_velocity = sensor.gyro


		imu_pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		publish_imu_msgs()

	except rospy ROSInterruptException as e:
		print('imu_streamer: ', e)