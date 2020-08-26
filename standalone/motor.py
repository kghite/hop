"""
Motor Controller (test only)
"""

import time
from gpiozero import Motor

if __name__ == '__main__':
	motor = Motor(24, 25)
	motor.forward()
	time.sleep(2)
	motor.stop()
	time.sleep(2)
	motor.backward(0.5)
	time.sleep(2)
	motor.stop()
