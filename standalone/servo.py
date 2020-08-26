"""
Servo Controller (test only)
"""

import time
from gpiozero import Servo


if __name__ == '__main__':
	l = Servo(7)
	l.min()
	time.sleep(5)
	l.max()
	time.sleep(1)
