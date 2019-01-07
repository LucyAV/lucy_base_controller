#include <math.h>

#include <ros/ros.h>
#include <std_msgs/UInt16.h>

void motor_control_callback(const std_msgs::UInt16& msg);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_control_debugger");
	ros::NodeHandle nodehandle;

	/* topic, buffer, (callback) */
	ros::Subscriber sub = nodehandle.subscribe("lucy/motor_control", 100, motor_control_callback);
	ros::spin();
}

void motor_control_callback(const std_msgs::UInt16& msg)
{
	uint16_t data = msg.data;

	uint16_t schub = data >> 8;
	uint16_t steering = data & 0x00FF;

	ROS_INFO("\nReceived: %u\n\tSpeed: %u\n\tSteering: %u", data, schub, steering);
}