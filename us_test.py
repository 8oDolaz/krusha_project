from sensors.ultrasonic_sensor import Ultra_Sonic

import time

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

TRIG_1 = 36
ECHO_1 = 38

us_1 = Ultra_Sonic(0, 0, 0, TRIG_1, ECHO_1)

try:
	while True:
		time.sleep(0.3)

		print(us_1.measure_distance())

except KeyboardInterrupt:
	GPIO.cleanup()
