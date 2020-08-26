"""
Hop Harware Interface
"""

from gpiozero import Motor, Servo

from led import Led
from camera import Camera
from imu import Imu


class Hop:

	def __init__(self, red_led_pin, green_led_pin,
					left_servo_pin, right_servo_pin,
					left_motor_forward, left_motor_backward,
					right_motor_forward, right_motor_backward):
		# Input
		self.imu = Imu()
		self.camera = Camera()

		# Output
		self.red_led = Led(red_led_pin)
		self.green_led = Led(green_led_pin)
		self.left_servo = Servo(left_servo_pin, min_angle=-90, max_angle=90)
		self.right_servo = Servo(right_servo_pin, min_angle=-90, max_angle=90)
		self.left_motor = Motor(forward=left_motor_forward, backward=left_motor_backward)
		self.right_servo = Motor(forward=right_motor_forward, backward=right_motor_backward)

	def pid_balance(self):
		pass

	def lqr_balance(self):
		pass

	def lqr_drive(self):
		pass

	def jump(self):
		pass
