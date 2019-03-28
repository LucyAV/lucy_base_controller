#!/usr/bin/env python
# Line above declares this document as being a python script

# Imports for ROS and message types
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

# Global publisher
publisher = None

# Forward clear state
is_forward_clear = False

def lidar_data_handler(data):
	global is_forward_clear

	# Store LiDAR data ranges
	ranges = data.ranges
	# Create array only for the sensor values in front of the vehicle
	values = [None] * 40

	# Fill sensor forward values array
	for i in range(0, 40):
		values[i] = ranges[160 + i]

	# Check if forward is clear based on different threshold values for the
	# different forward value ranges
	currentLowDistCount = 0
	for i in range(0, 10):
		if (values[i] != float('Inf')) and (values[i] <= 1.0):
			currentLowDistCount += 1

	for i in range(10, 30):
		if (values[i] != float('Inf')) and (values[i] <= 1.5):
			currentLowDistCount += 1

	for i in range(30, 40):
		if (values[i] != float('Inf')) and (values[i] <= 1.0):
			currentLowDistCount += 1

	# If more than one measured value has been detected as being low distance,
	# there is an obstacle in front of the car
	if currentLowDistCount >= 2:
		is_forward_clear = False
	else:
		is_forward_clear = True

	# Publish the forward clear state
	publisher.publish(is_forward_clear)

def lidar_data_receiver_setup():
	# Initialize the publisher of the forward clear state
	global publisher
	publisher = rospy.Publisher('lucy/lidar_forward_clear', Bool, queue_size=10)

	# Initialize this node
	rospy.init_node('lidar_controller', anonymous=True)

	# Subscribe to the LiDAR scan data and assign a handler for newly received data
	rospy.Subscriber('/scan', LaserScan, lidar_data_handler)

	# Keep this software running so that the listener can still be called
	rospy.spin()
	
	print("Stopped lidar_controller")

if __name__ == '__main__':
	try:
		print("Starting lidar_controller")

		# Call setup method
		lidar_data_receiver_setup()
	except rospy.ROSInterruptException:
		print("Exiting lidar_controller")
		pass
