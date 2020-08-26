"""
Standalone Hop Controller (no ROS version)
"""

import time
import configparser
from enum import Enum

from hop import Hop


class State(Enum):
	booting = 1
	standing = 2
	stable = 3
	teleoping = 4
	executing = 5
	fallen = 6
	estopped = 7
	stopping = 8
	testing = 9


class FSM:

	def __init__(self, robot):
		self.state = booting
		self.prev_state = None

		self.robot = robot

		self.boot_loops = 20

	"""
	Boot the robot
	"""
	def boot(self):
		# Initial loop
		if self.prev_state != self.state:
			self.robot.red_led.off()
			self.robot.green_led.blink()
			print("Booting")
			self.prev_state = self.state
			
		if self.boot_loops > 0:
			self.boot_loops-=1
		else:
			# Exit case
			self.state = State.standing

	"""
	Stand up from a ground state
	"""
	def stand(self):
		# Initial loop
		if self.prev_state != self.state:
			self.robot.red_led.blink()
			self.robot.green_led.on()
			print('Standing')
			self.prev_state = self.state

	"""
	Balance in a stable standing state
	"""
	def balance(self):
		# Initial loop
		if self.prev_state != self.state:
			self.robot.red_led.off()
			self.robot.green_led.on()
			print('Balancing')
			self.prev_state = self.state

	"""
	Move to follow teleop while balancing
	"""
	def teleop(self):
		# Initial loop
		if self.prev_state != self.state:
			self.robot.red_led.blink()
			self.robot.green_led.on()
			print('Teleoping')
			self.prev_state = self.state

	"""
	Execute file of custom mission states
	"""
	def execute(self):
		# Initial loops
		if self.prev_state != self.state:
			self.robot.red_led.blink()
			self.robot.green_led.on()
			print('Executing Mission')
			self.prev_state = self.state

	"""
	React to a fall
	"""
	def recover(self):
		# Initial loop
		if self.prev_state != self.state:
			self.robot.red_led.blink()
			self.robot.green_led.off()
			print('Recovering from Fall')
			self.prev_state = self.state

	"""
	Handle an estop from safety check or hardware switch
	"""
	def estop(self):
		# Initial loop
		if self.prev_state != self.state:
			self.robot.red_led.on()
			self.robot.green_led.off()
			print('Emergency Stopping')
			self.prev_state = self.state

	"""
	Shut down the robot
	"""
	def stop(self):
		# Initial loop
		if self.prev_state != self.state:
			self.robot.red_led.blink()
			self.robot.green_led.on()
			print('Shutting Down')
			self.prev_state = self.state

		# Exit case (lights cleaned up last)
		self.robot.red_led.reset()
		self.robot.green_led.reset()
		self.state = None

	"""
	Testing state for development
	"""
	def test(self):
		# Initial loop
		if self.prev_state != self.state:
			self.robot.red_led.blink()
			self.robot.green_led.on()
			print('Testing')
			self.prev_state = self.state


if __name__ == '__main__':
	# Get robot config
	config = configparser.ConfigParser()
	config.read('example.ini')

	# Set up controller
	c = config['Expression']
	robot = Hop(red_led_pin=c[r_led], green_led_pin=c[g_led],
					left_servo_pin=c[l_servo], right_servo_pin=c[r_servo],
					left_motor_forward=c[l_motor_f], left_motor_backward=c[l_motor_b],
					right_motor_forward=c[r_motor_f], right_motor_backward=c[r_motor_b])
	fsm = FSM(robot)

	# Run controller at 10hz
	while(True):
		if fsm.state is State.booting:
			fsm.boot()
		elif fsm.state is State.standing:
			fsm.stand()
		elif fsm.state is State.stable:
			fsm.balance()
		elif fsm.state is State.teleoping:
			fsm.teleop()
		elif fsm.state is State.executing:
			fsm.execute()
		elif fsm.state is State.fallen:
			fsm.recover()
		elif fsm.state is State.estopped:
			fsm.estop()
		elif fsm.state is State.stopping:
			fsm.stop()
		elif fsm.state is State.testing:
			fsm.test()
		else:
			print("Controller shutdown complete")
			break

		time.sleep(0.1)