#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "geometry_msgs/Twist.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd_vel_to_steering_and_speed");

	ros::NodeHandle nodehandle;

	/* topic, buffer, callback */
	ros::Subscriber sub = nodehandle.subscribe("cmd_vel", 10, chatterCallback);

	ros::spin();

}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
