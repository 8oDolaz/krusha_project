import math
import time

import RPi.GPIO as GPIO

FOV = 15  # Field of view in degrees (constant)
MAX_RANGE = 200  # Max range for sensor in meters.

#Class for sensor HC-SR004
class Ultra_Sonic:
	#It has two parameters - pin numbers (integer)
	#	TRIG - the output signal to our sensor
	#	ECHO - the input signal from our sensor

	def __init__(self, x, y, theta, output_pin, input_pin):
		self.x, self.y, self.theta = x, y, theta

		self.TRIG = output_pin
		self.ECHO = input_pin

		GPIO.setup(self.TRIG, GPIO.OUT, initial=False)
		GPIO.setup(self.ECHO, GPIO.IN)


	def measure_distance(self):
		# Calculate the distance. Without parameters 

		#Creating a signal on our sensor
		GPIO.output(self.TRIG, True)
		time.sleep(0.00001)
		GPIO.output(self.TRIG, False)

		#Waiting for a response signal
		while GPIO.input(self.ECHO) == 0:
			pulse_start = time.time()

		while GPIO.input(self.ECHO) == 1:
			pulse_end = time.time()

		#Calculate the distance 
		pulse_duration = pulse_end - pulse_start
		distance = pulse_duration * 17150

		return distance

	def calculate_covariance(self, distance):
		# Calculate the covariance.
		global FOV
		delta_x = distance * math.tan(math.radians(FOV / 2))
		covariance = delta_x ** 2

		return covariance