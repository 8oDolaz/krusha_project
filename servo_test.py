import RPi.GPIO as GPIO
import time 
GPIO.setmode(GPIO.BOARD)

logic = 36
GPIO.setup(logic, GPIO.OUT)

servol = GPIO.PWM(logic, 50)

servol.start(0)
time.sleep(2)

step = 0
flag = 1
try:
	while (True):
		step += flag

		if step % 12 == 0:
			flag *= -1

		servol.ChangeDutyCycle(step)

		print(step)
		time.sleep(0.1)
except KeyboardInterrupt:
	servol.stop()
	GPIO.cleanup()
