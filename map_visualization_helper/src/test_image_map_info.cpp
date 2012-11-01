#include <ros/ros.h>
#include <mobots_msgs/PoseAndID.h>
#include <mobots_msgs/IDKeyValue.h>
#include <map_visualization/RemoteProcedureCall.h>

ros::NodeHandle nh;
ros::Subscriber poseRelayTestSub;
ros::Publisher updateInfoTestPub;
ros::ServiceServer updateRvizTestServer;

void poseRelayTestHandler(const mobots_msgs::PoseAndID::ConstPtr& msg){
    ROS_INFO("Pose(%f, %f, %f) ID(%i, %i, %i)", msg->pose.x, msg->pose.y,
             msg->pose.theta, msg->id.session_id, msg->id.mobot_id, msg->id.image_id);
    return 1;
}

int updateInfoTest(){
    return 1;
}

int updateRvizTest(){
    return 1;
}

int main(int argc, char** argv){
    ros::init(argc, argv);
    // Test input: User Waypoint from Rviz
    poseRelayTestSub = nh.subscribe("/path_planner/waypoint_user", 10,
            poseRelayTestHandler);
    updateInfoTestPub = nh.advertise
            <mobots_msgs::IDKeyValue>("/image_map/update_push", 10);
    updateRvizTestServer = nh.advertise("/image_map/rpc", updateRvizTestHandler);
    if(argv[1][0] == 'a'){
        return 1;
    }
    if(argv[1][0] == 'b'){
        return 1;
    }
    if(argv[1][0] == 'c'){
        return 1;
    }
    if(argv[1][0] == 'd'){
        return 1;
    }
    ROS_INFO("usage: 'a' for poseRelayTest, 'b' for updateInfo, 'c' for updateRviz, 'd' for all");
}
