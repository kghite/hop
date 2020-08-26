"""
IMU Controller
"""

import time
import board
import busio
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33


class IMU:

	def __init__(self):
		i2c = busio.I2C(board.SCL, board.SDA)
		self.sensor = LSM6DS33(i2c)

	"""
	Get acceleration reading - TODO: filter
	Return:
		* accel - [x, y, z]
	"""
	def get_accel(self):
		return self.sensor.acceleration

	"""
	Get gyroscope reading - TODO: filter
	Return:
		* gyro - [x, y, z]
	"""
	def get_gyro(self):
		return self.sensor.gyro

	"""
	Get n IMU readings at rate hz
	Input: 
		* n - number of readings
		* hz - frequency (max 1000)
	Return:
		* readings - array length n
	"""
	def get_stream(self, n, hz, show=False):
		period = 1.0 / float(hz)
		readings = []
		for _ in range(n):
			time_before = time.time()
			while (time.time() - time_before) < period:
				time.sleep(0.001)
			readings.append(self.sensor)
			if show:
				print("IMU accel: ", self.sensor.acceleration)
				print("IMU gyro: ", self.sensor.gyro)

		return readings


if __name__ == '__main__':
	# Test imu stream
	imu = IMU()
	imu.get_stream(10, 1, show=True)
