#include "ros/ros.h"

/**
 * Framework for a publisher
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "toro_server_test");
	ros::NodeHandle handle;
	ros::Publisher pub = handle.publish("config", 1000, configHandler);
	
	ros::Rate loop_rate(1);
	
	while (handle.ok())
	{
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}