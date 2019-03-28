#!/usr/bin/env python
# Line above declares this document as being a python script

# Imports for ROS and message types
import rospy
from std_msgs.msg import UInt16

# Imports for and setup of WiringPi (keep in this order)
import sys
sys.path.append('/home/ubuntu/.local/lib/python2.7/site-packages')
import wiringpi

# PWM channel pins
PWM_PIN_SERVO = 18
PWM_PIN_MOTOR = 19

# Values for the servo
servo_straight = 286
servo_variance = 86.0

# Values for the motor
motor_idle = 300
motor_variance = 28.0
motor_offset = 20
motor_current_value = motor_idle

def motor_data_handler(data):
	# data UInt16 description:
	# higher 8 bit is motor value
	# lower 8 bit is servo value

	global motor_current_value

	# Split and store the received data values for motor and servo
	received_data = data.data
	motor_received_value = received_data >> 8
	servo_received_value = received_data & 0x00FF

	# Calculate the next value to be applied to the motor
	if motor_received_value == 50:
		motor_current_value = motor_idle
	elif motor_received_value > 50:
		motor_current_value = (motor_idle + motor_offset) +\
		int( (motor_received_value - 50) / (50.0 / motor_variance) ) - 1
	elif motor_received_value < 50:
		if motor_current_value > 300:
			wiringpi.pwmWrite(PWM_PIN_MOTOR, 250)
			rospy.sleep(0.15)
			wiringpi.pwmWrite(PWM_PIN_MOTOR, 300)
			rospy.sleep(0.1)
		motor_current_value = (motor_idle - motor_offset) +\
		int( (motor_received_value - 50) / (50.0 / motor_variance) )
	
	# Calculate and apply servo value
	servo_current_value = servo_straight +
	int( (servo_received_value - 50) / (50.0 / servo_variance) )
	wiringpi.pwmWrite(PWM_PIN_SERVO, servo_current_value)
	
	# Apply motor value
	wiringpi.pwmWrite(PWM_PIN_MOTOR, motor_current_value)

def pwm_setup():
	# Clock variables
	PWM_CLOCK = 96;
	PWM_RANGE = 4000;

	# Initialize WiringPi, then Pin, then PWM Mode for servo
	wiringpi.wiringPiSetupGpio()
	wiringpi.pinMode(PWM_PIN_SERVO, wiringpi.PWM_OUTPUT)
	wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
	# Configure Frequency
	wiringpi.pwmSetClock(PWM_CLOCK)
	wiringpi.pwmSetRange(PWM_RANGE)
	wiringpi.pwmWrite(PWM_PIN_SERVO, 282)

	# Initialize WiringPi, then Pin, then PWM Mode for motor
	wiringpi.wiringPiSetupGpio()
	wiringpi.pinMode(PWM_PIN_MOTOR, wiringpi.PWM_OUTPUT)
	wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
	# Configure Frequency
	wiringpi.pwmSetClock(PWM_CLOCK)
	wiringpi.pwmSetRange(PWM_RANGE)
	wiringpi.pwmWrite(PWM_PIN_MOTOR, 300)

def motor_data_receiver_setup():
	# Initialize this node
	rospy.init_node('motor_controller', anonymous=True)

	# Subscribe to the motor control data and assign a handler for newly
	# received data coming from either the manual or the autonomous control
	rospy.Subscriber('lucy/motor_control', UInt16, motor_data_handler)

if __name__ == '__main__':
	try:
		print("Starting motor_controller")

		# Call setup methods
		motor_data_receiver_setup()
		pwm_setup()

		# Keep this software running so that the listener can still be called
		rospy.spin()
	except rospy.ROSInterruptException:
		print("Exiting motor_controller")
		pass
