#!/usr/bin/env python

# Imports for ROS
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Bool

# Import and setup for WiringPi (keep order)
import sys
sys.path.append('/home/ubuntu/.local/lib/python2.7/site-packages')
import wiringpi

# PWM pins
PWM_PIN_SERVO = 18
PWM_PIN_MOTOR = 19

# Values for the servo
servo_straight = 286
servo_variance = 86.0
#245 to 355

# Values for the motor
motor_idle = 300
motor_variance = 28.0
motor_offset = 20
motor_current = motor_idle

# State of forward clearance
lidar_forward_clear_current = True
lidar_forward_clear_hasSwiched = False

def motor_data_handler(data):
	# higher 8 bit is motor value
	# lower 8 bit is servo value
	global motor_current

	# Split and store the received data values for motor and servo
	received_data = data.data
	motor_received_value = received_data >> 8
	servo_received_value = received_data & 0x00FF

	# Store the previous actual value of the motor
	motor_previous = motor_current
	
	# Calculate/Determine the next value to be applied to the motor
	if motor_received_value == 50:
		motor_current = motor_idle
	elif motor_received_value > 50:
		motor_current = (motor_idle + motor_offset) + int( (motor_received_value - 50) / (50.0 / motor_variance) ) - 1
	elif motor_received_value < 50:
		if motor_current > 300:
			wiringpi.pwmWrite(PWM_PIN_MOTOR, 250)
			rospy.sleep(0.15)
			wiringpi.pwmWrite(PWM_PIN_MOTOR, 300)
			rospy.sleep(0.1)
		motor_current = (motor_idle - motor_offset) + int( (motor_received_value - 50) / (50.0 / motor_variance) )

	# Check if forward is clear and apply motor value if so
	if lidar_forward_clear_current:
		wiringpi.pwmWrite(PWM_PIN_MOTOR, motor_current)
	else:
		# Also apply motor value if going backwards/braking
		if motor_current <= 300:
			wiringpi.pwmWrite(PWM_PIN_MOTOR, motor_current)
		if motor_previous > 300:
			# If the forward clear just previously changed to false, then brake
			if lidar_forward_clear_hasSwiched is True:
				wiringpi.pwmWrite(PWM_PIN_MOTOR, 250)
				rospy.sleep(0.9) # 0.15
				wiringpi.pwmWrite(PWM_PIN_MOTOR, 300)
				rospy.sleep(0.1)
				motor_current = motor_idle

				# Reset the lidar_forward_clear_hasSwitched variable, as its action has been fulfilled
				global lidar_forward_clear_hasSwiched
				lidar_forward_clear_hasSwiched = False
				wiringpi.pwmWrite(PWM_PIN_MOTOR, motor_current)
			# Only if the previous value was greater than 300 and the current value is smaller than 325, then apply the motor value as well
			elif motor_current < 325:
				wiringpi.pwmWrite(PWM_PIN_MOTOR, motor_current)
			

	# Calculate and apply servo value
	servo_current_value = servo_straight + int( (servo_received_value - 50) / (50.0 / servo_variance) )
	wiringpi.pwmWrite(PWM_PIN_SERVO, servo_current_value)
	

def pwm_setup():
	# Clock variables
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

def lidar_forward_clear_handler(data):
	lidar_forward_clear_received = data.data

	# Check if the lidar forward clearance has changed with its last update
	if (lidar_forward_clear_received is False) and (lidar_forward_clear_current is True):
		global lidar_forward_clear_hasSwiched
		lidar_forward_clear_hasSwiched = True

	# Save the current lidar forward clear state
	global lidar_forward_clear_current
	lidar_forward_clear_current = data.data

def motor_data_receiver_setup():
	rospy.init_node('motor_controller', anonymous=True)
	rospy.Subscriber('lucy/motor_control', UInt16, motor_data_handler)
	rospy.Subscriber('/lucy/lidar_forward_clear', Bool, lidar_forward_clear_handler)

if __name__ == '__main__':
	try:
		print("Starting stop_controller")
		motor_data_receiver_setup()
		pwm_setup()
		rospy.spin()
	except rospy.ROSInterruptException:
		print("Exiting")
		pass
