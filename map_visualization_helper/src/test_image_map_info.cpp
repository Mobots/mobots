#include <stdio.h>

#include <ros/ros.h>
#include <mobots_msgs/PoseAndID.h>
#include <mobots_msgs/IDKeyValue.h>
#include <map_visualization/RemoteProcedureCall.h>
#include <iostream>
#include <map_visualization/definitions.h>

ros::NodeHandle* nh;
ros::Subscriber* poseRelayTestSub;
ros::Publisher* updateInfoTestPub;
ros::ServiceServer* updateRvizTestServer;

void poseRelayTestHandler(const mobots_msgs::PoseAndID::ConstPtr& msg){
    ROS_INFO("Pose(%f, %f, %f) ID(%i, %i, %i)", msg->pose.x, msg->pose.y,
             msg->pose.theta, msg->id.session_id, msg->id.mobot_id, msg->id.image_id);
}

void updateInfoTest(){
    ROS_INFO("updateInfoTest");
    mobots_msgs::IDKeyValue msg;
    msg.id.session_id = 0;
    msg.id.mobot_id = 0;
    msg.id.image_id = 0;
    int a;
    std::cout << "Enter the session:" << std::endl;
    std::cin >> a;
    msg.id.session_id = a;
    std::cout << "Enter the mobot:" << std::endl;
    std::cin >> a;
    msg.id.mobot_id = a;
    while(ros::ok()){
        msg.id.mobot_id %= 3;
        std::cout << "Enter the key:" << std::endl;
        std::cin >> a;
        msg.key = a;
        std::cout << "Enter the value:" << std::endl;
        std::cin >> a;
        msg.value = a;
        updateInfoTestPub->publish(msg);
        ROS_INFO("spin");
        ros::spinOnce();
        msg.id.mobot_id++;
    }
    ROS_INFO("updateInfoTest");
}

bool updateRvizTest(map_visualization::RemoteProcedureCall::Request &req,
                    map_visualization::RemoteProcedureCall::Response &res){
    ROS_INFO("[updateRvizTest]");
    int function = req.function;
    switch(function){
    case INSERT_IMAGE:
        ROS_INFO("a");
        break;
    case SHOW_IMAGE:
        ROS_INFO("b");
        break;
    case HIDE_IMAGE:
        ROS_INFO("c");
        break;
    case DELETE_IMAGE:
        ROS_INFO("d");
        break;

    case SHOW_MOBOT:
        ROS_INFO("e");
        break;
    case HIDE_MOBOT:
        ROS_INFO("f");
        break;
    case DELETE_MOBOT:
        ROS_INFO("g");
        break;

    case SHOW_SESSION:
        ROS_INFO("h");
        break;
    case HIDE_SESSION:
        ROS_INFO("i");
        break;
    case DELETE_SESSION:
        ROS_INFO("j");
        break;

    case DELETE_ALL_IMAGES:
        ROS_INFO("k");
        break;

    case SET_IMAGE_POSE:
        ROS_INFO("l");
        break;

    case DELETE_MOBOT_MODEL:
        ROS_INFO("m");
        break;
    case DELETE_ALL_MOBOT_MODELS:
        ROS_INFO("n");
        break;
    case SET_MOBOT_MODEL:
        ROS_INFO("o");
        break;
    }
    res.result = 0;
    return true;
}

int main(int argc, char** argv){
    ROS_INFO("Test");
    ros::init(argc, argv, "map_visualization_test");
    ros::NodeHandle nh_;
    nh = &nh_;
    ROS_INFO("Test");
    // Test input: User Waypoint from Rviz
    ros::Subscriber poseRelayTestSub_ = nh->subscribe("/path_planner/waypoint_user", 10,
                                    poseRelayTestHandler);
    poseRelayTestSub = &poseRelayTestSub_;
    ros::Publisher updateInfoTestPub_ = nh->advertise
            <mobots_msgs::IDKeyValue>("/image_map/update_push", 10);
    updateInfoTestPub = &updateInfoTestPub_;
    ros::ServiceServer updateRvizTestServer_ = nh->advertiseService("/image_map/rpc", updateRvizTest);
    updateRvizTestServer = &updateRvizTestServer_;
    ROS_INFO("Test");
    if(argc == 2){
        if(argv[1][0] == 'a'){
            ROS_INFO("Testa");
            updateInfoTest();
            return 1;
        }
    }
    ros::spin();
    return 0;
}
