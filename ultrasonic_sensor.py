import time

import RPi.GPIO as GPIO


class Ultra_Sonic:
	def __init__(self, output_pin: int, input_pin: int) -> None:
		self.TRIG = output_pin
		self.ECHO = input_pin

		GPIO.setup(self.TRIG, GPIO.OUT, initial=False)
		GPIO.setup(self.ECHO, GPIO.IN)


	def measure_distance(self) -> float:
		GPIO.output(self.TRIG, True)
		time.sleep(0.00001)
		GPIO.output(self.TRIG, False)

		while GPIO.input(self.ECHO) == 0:
			pulse_start = time.time()

		while GPIO.input(self.ECHO) == 1:
			pulse_end = time.time()

		pulse_duration = pulse_end - pulse_start

		distance = pulse_duration * 17150

		return distance
