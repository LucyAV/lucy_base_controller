#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

void cmd_vel_callback(const geometry_msgs::Twist& msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd_vel_to_steering_and_speed");

	ros::NodeHandle nodehandle;

	/* topic, buffer, callback */
	ros::Subscriber sub = nodehandle.subscribe("cmd_vel", 10, cmd_vel_callback);

	ros::spin();

}

void cmd_vel_callback(const geometry_msgs::Twist& msg)
{
	ROS_INFO("Peta: [%f]", msg.linear.x);
}