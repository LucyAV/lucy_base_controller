#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>

void cmd_vel_callback(const geometry_msgs::Twist& msg);

const float MAX_SCHUB_FLOAT = 1.0f;
const float MAX_STEERING_FLOAT = 1.0f;
const uint8_t MAX_SCHUB_INT = 50;
const uint8_t ZERO_SCHUB_INT = 50; /* schub 0 - 100, where 50 is zero */
const uint8_t MAX_STEERING_INT = 50;
const uint8_t ZERO_STEERING_INT = 50; /* steering 0 - 100, where 50 is zero */

ros::Publisher publisher;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd_vel_to_steering_and_speed");

	ros::NodeHandle nodehandle;

	/* topic, buffer, (callback) */
	ros::Subscriber sub = nodehandle.subscribe("cmd_vel", 10, cmd_vel_callback);
	publisher = nodehandle.advertise<std_msgs::UInt16>("lucy/motor_control", 10);


	ROS_INFO("Publishing to lucy/motor_control...");

	ros::spin();

}

void cmd_vel_callback(const geometry_msgs::Twist& msg)
{
	//ROS_INFO("Peta: [%f] [%f]", msg.linear.x, msg.angular.z);

	float schubOld = msg.linear.x; 
	float steeringOld = msg.angular.z;

	bool negativeSchub = schubOld < 0.0f;
	bool negativeSteering = steeringOld < 0.0f;

	schubOld = fabs(schubOld);
	steeringOld = fabs(steeringOld);

	if(schubOld > MAX_SCHUB_FLOAT) schubOld = MAX_SCHUB_FLOAT;
	if(steeringOld > MAX_STEERING_FLOAT) steeringOld = MAX_STEERING_FLOAT;

	/* normalize to 1 */
	schubOld = schubOld / MAX_SCHUB_FLOAT;
	steeringOld = steeringOld / MAX_SCHUB_FLOAT;

	/* normalize to MAX_SCHUB_INT and MAX_STEERING_INT */
	schubOld = schubOld * MAX_SCHUB_INT;
	steeringOld = steeringOld * MAX_STEERING_INT;

	uint16_t schub, steering;

	if(negativeSchub)
	{
		schub = ZERO_SCHUB_INT - schubOld;
	}
	else
	{
		schub = ZERO_SCHUB_INT + schubOld;
	}

	if(negativeSteering)
	{
		steering = ZERO_STEERING_INT + steeringOld;
	}
	else
	{
		steering = ZERO_STEERING_INT - steeringOld;
	}

	uint16_t data;

	data = steering;
	data = (schub << 8) | data;

	std_msgs::UInt16 message;

	message.data = data;
	publisher.publish(message);
}