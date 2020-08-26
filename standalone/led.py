"""
LED Controller (Async)
"""

import RPi.GPIO as GPIO
import time
import threading


class Led(object):

	LED_OFF = 0
	LED_ON = 1
	LED_FLASHING = 2

	FAST_CYCLE = 0.05

	def __init__(self, led_pin):
		self.pin_stop = threading.Event()

		# Init led pin and set OFF
		self.led_pin = led_pin
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.led_pin, GPIO.OUT)
		self.ledmode = Led.LED_OFF
		self.off()

		# Create thread
		self.thread = threading.Thread(name='ledblink',target=self.__blink_pin)
		self.thread.start()

	"""
	Blink LED starting at next first period
	"""
	def blink(self, time_on=0.050, time_off=1):
		self.ledmode = Led.LED_FLASHING
		self.__time_on = time_on
		self.__time_off = time_off

	"""
	Turn off the LED - TODO: Fix flicker from LED_FLASHING transition
	"""
	def off(self):
		self.ledmode = self.LED_OFF
		self.__time_on = Led.FAST_CYCLE
		self.__time_off = Led.FAST_CYCLE

	"""
	Turn on the LED - TODO: Fix flicker from LED_FLASHING transition
	"""
	def on(self):
		self.ledmode = self.LED_ON
		self.__time_on = Led.FAST_CYCLE
		self.__time_off = Led.FAST_CYCLE

	"""
	Reset and clean thread
	"""
	def reset(self):
		# Exit thread after sleep
		self.pin_stop.set()
		self.thread.join()

		# Clean GPIO
		GPIO.cleanup()

	#### PRIVATE ####

	def __turnledon(self, pin):
		GPIO.output(pin, GPIO.LOW)

	def __turnledoff(self, pin):
		GPIO.output(pin, GPIO.HIGH)

	def __blink_pin(self):
		while not self.pin_stop.is_set():
			if self.ledmode == Led.LED_ON or self.ledmode == Led.LED_FLASHING: 
				self.__turnledon()
			else:
				self.__turnledoff()

			time.sleep(self.__time_on)

			if self.ledmode == Led.LED_FLASHING:
				self.__turnledoff()
				if not self.pin_stop.is_set():
					time.sleep(self.__time_off)


if __name__ == '__main__':
	# Test led modes
	red = Led(5)
	red.on()
	time.sleep(5)
	red.blink()
	time.sleep(5)
	red.off()