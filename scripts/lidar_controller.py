#!/usr/bin/env python

# Imports for ROS
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

# Global publisher
publisher = None

# Forward clearance status
is_forward_clear = False

def lidar_data_handler(data):
	global is_forward_clear

	# Store ranges
	ranges = data.ranges
	# Create array for sensor forward values
	values = [None] * 40

	# Fill sensor forward values array
	for i in range(0, 40):
		values[i] = ranges[160 + i]

	# Check if forward is clear based on different threshold values for the different forward values
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

	if currentLowDistCount >= 2:
		is_forward_clear = False
	else:
		is_forward_clear = True

	# Publish the forward clear state
	publisher.publish(is_forward_clear)

def lidar_data_receiver_setup():
	global publisher
	publisher = rospy.Publisher('lucy/lidar_forward_clear', Bool, queue_size=10)
	rospy.init_node('lidar_controller', anonymous=True)
	rospy.Subscriber('/scan', LaserScan, lidar_data_handler)

	rospy.spin()
		
	print("Stopped lidar_controller")

if __name__ == '__main__':
	try:
		print("Starting lidar_controller")
		lidar_data_receiver_setup()
	except rospy.ROSInterruptException:
		print("Exiting")
		pass
