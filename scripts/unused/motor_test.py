#!/usr/bin/env python

# Imports for ROS
import rospy

# Import and setup for WiringPi (keep order)
import sys
sys.path.append('/home/ubuntu/.local/lib/python2.7/site-packages')
import wiringpi

# PWM pins
PWM_PIN_SERVO = 18
PWM_PIN_MOTOR = 19

def pwm_setup():
	# Clock variables
	#PWM_CLOCK = 192;
	#PWM_RANGE = 2000;
	PWM_CLOCK = 96;
	PWM_RANGE = 4000;

	# Initialize WiringPi, then Pin, then PWM Mode
	wiringpi.wiringPiSetupGpio()
	wiringpi.pinMode(PWM_PIN_SERVO, wiringpi.PWM_OUTPUT)
	wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
	# Configure Frequency
	wiringpi.pwmSetClock(PWM_CLOCK)
	wiringpi.pwmSetRange(PWM_RANGE)
	wiringpi.pwmWrite(PWM_PIN_SERVO, 282)

	# Initialize WiringPi, then Pin, then PWM Mode
	wiringpi.wiringPiSetupGpio()
	wiringpi.pinMode(PWM_PIN_MOTOR, wiringpi.PWM_OUTPUT)
	wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
	# Configure Frequency
	wiringpi.pwmSetClock(PWM_CLOCK)
	wiringpi.pwmSetRange(PWM_RANGE)
	wiringpi.pwmWrite(PWM_PIN_MOTOR, 300)

	print("Setup done, idle for 2 s")
	rospy.sleep(2)

	#316 as minimum forward
	#279 as minimum reverse

	wiringpi.pwmWrite(PWM_PIN_MOTOR, 324)
	print("324 for 2s")
	rospy.sleep(1)

	wiringpi.pwmWrite(PWM_PIN_MOTOR, 280)
	print("280 for 0.1s")
	rospy.sleep(0.1)

	wiringpi.pwmWrite(PWM_PIN_MOTOR, 300)
	print("300 for 0.1s")
	rospy.sleep(0.1)

	wiringpi.pwmWrite(PWM_PIN_MOTOR, 278)
	print("278 for 1.5s")
	rospy.sleep(1.5)

	wiringpi.pwmWrite(PWM_PIN_MOTOR, 300)

if __name__ == '__main__':
	try:
		print("Starting motor_test")
		pwm_setup()
	except rospy.ROSInterruptException:
		print("Exiting")
		pass
