"""
Standalone Hop Controller (no ROS version)
"""

import time
from enum import Enum

from hop import Hop


class State(Enum):
	booting = 1
	standing = 2
	stable = 3
	fallen = 4
	estopped = 5
	stopping = 6
	testing = 7


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
	robot = Hop(red_led_pin=5, green_led_pin=6,
					left_servo_pin=7, right_servo_pin=8,
					left_motor_forward=18, left_motor_backward=23,
					right_motor_forward=24, right_motor_backward=25)
	fsm = FSM(robot)

	# Run mission controller at 10hz
	while(True):
		if fsm.state is State.booting:
			fsm.boot()
		elif fsm.state is State.standing:
			fsm.stand()
		elif fsm.state is State.stable:
			fsm.balance()
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