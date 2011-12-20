#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>

ros::ServiceClient client;
ros::ServiceClient getClient;

void teleopCallback(const geometry_msgs::Twist &twistMsg){
	gazebo_msgs::GetModelState getState;
	getState.request.model_name = std::string("mobot");
	getState.request.relative_entity_name = std::string("world");
	bool result = getClient.call(getState);

	gazebo_msgs::ModelState modelState;
	modelState.model_name = std::string("mobot");
	modelState.reference_frame = std::string("world"); 

	modelState.twist = twistMsg;

	modelState.pose = getState.response.pose;
	
	gazebo_msgs::SetModelState setModelState; 
	setModelState.request.model_state = modelState;
	result = client.call(setModelState);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "mobots_teleop_bridge");
	ros::NodeHandle n;
	client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); //true für persistente Verbindung <> GEHT NICHT IN DIESEM FALL >_<
	client.waitForExistence();
	getClient = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true);
	getClient.waitForExistence();
	ros::Subscriber sub = n.subscribe("/cmd_vel", 100, teleopCallback);  //wichtig: Wenn das zurückgegebene Objekt nicht mehr referenziert wird, wird der tatsächliche Subscriber zerstört
	ros::spin();
}
