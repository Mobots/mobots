#include <iostream>
#include <ros/ros.h>
#include <signal.h>

#include "sensor_msgs/Joy.h"
#include "path_planner/KeyboardRequest.h"
#include "mobots_msgs/Twist2D.h"

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

using namespace std;

ros::Publisher velocity_pub;
ros::Subscriber joySub;
ros::ServiceClient client;
ros::NodeHandle* nh;
int mobotID;
int indexRight;

void siginthandler(int param){
	path_planner::KeyboardRequest::Request req;
	path_planner::KeyboardRequest::Response res;
	req.mobot_id = mobotID;
	req.enable = false;
	if(!client.call(req, res))
		ROS_WARN("[mobot_sixaxis_teleop] No path_planner found while detaching teleop");
	
	mobots_msgs::Twist2D msg;
	msg.x = 100;
	msg.y = 100;
	msg.theta = 100;
	velocity_pub.publish(msg);
	exit(1);
}

void joyCallback(const sensor_msgs::Joy& msg);

int main(int argc, char** argv){
	cout << "specify a mobot to control (enter the mobot id): ";
	cin >> mobotID;
	cout << endl;
	cout << "index for right stick? (sixaxis = 2, davids shit = 3):";
	cin >> indexRight;
	cout << endl;
	std::stringstream namess;
	namess << "mobot_sixaxis_teleop_" << mobotID;
	ros::init(argc, argv, namess.str(), ros::init_options::NoSigintHandler);
  nh = new ros::NodeHandle;
	joySub = nh->subscribe("/joy", 2, joyCallback);
	cout << "searching for sixaxis driver..." << flush;
	sleep(1);
	if(joySub.getNumPublishers() < 1){
		cout << "no sixaxis driver publishing on /joy topic found" << endl;
		exit(1);
	}
	cout << "ok" << endl;
	
	client = nh->serviceClient<path_planner::KeyboardRequest>("/path_planner/keyboard_request");
	path_planner::KeyboardRequest::Request req;
	path_planner::KeyboardRequest::Response res;
	req.mobot_id = mobotID;
	req.enable = true;
	if(client.call(req, res)){
		if(!res.enabled){
			cout << "path_planner does currently not allow keyboard controlling for this id" << endl;
			exit(1);
		}
	}else{
		ROS_WARN("[%s] No path_planner found while attaching teleop", namess.str().c_str());
	}
	stringstream ss;
	ss << "/mobot" << mobotID << "/velocity";
	velocity_pub = nh->advertise<mobots_msgs::Twist2D>(ss.str(), 2);
	signal(SIGINT, siginthandler);

	ros::spin();

  return(0);
}

mobots_msgs::Twist2D twist;
mobots_msgs::Twist2D lastTwist;

void joyCallback(const sensor_msgs::Joy& msg){
	twist.x = -msg.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
	twist.y = msg.axes[PS3_AXIS_STICK_LEFT_UPWARDS];
	twist.theta = -msg.axes[indexRight];
	if(twist.x == lastTwist.x && twist.y == lastTwist.y && twist.theta == lastTwist.theta)
		return;
	velocity_pub.publish(twist);
	lastTwist = twist;
	//cout << "x " << twist.x << " y " << twist.y << " theta " << twist.theta << endl;
}
