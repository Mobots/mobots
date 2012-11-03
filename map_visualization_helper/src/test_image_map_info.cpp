#include <stdio.h>

#include <ros/ros.h>
#include <mobots_msgs/PoseAndID.h>
#include <mobots_msgs/IDKeyValue.h>
#include <map_visualization/RemoteProcedureCall.h>
#include <iostream>
#include <definitions.h>

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
    std::cout << "Enter the session:" << std::endl;
    std::cin >> msg.id.session_id;
    std::cout << "Enter the value:" << std::endl;
    std::cin >> msg.id.mobot_id;
    ros::Rate rate_limit(1);
    while(ros::ok()){
        msg.id.mobot_id %= 3;
        std::cout << "Enter the key:" << std::endl;
        std::cin >> msg.key;
        std::cout << "Enter the value:" << std::endl;
        std::cin >> msg.value;
        updateInfoTestPub->publish(msg);
        ROS_INFO("spin");
        ros::spinOnce();
        rate_limit.sleep();
        msg.id.mobot_id++;
    }
    ROS_INFO("updateInfoTest");
}

bool updateRvizTest(map_visualization::RemoteProcedureCall::Request &req,
                    map_visualization::RemoteProcedureCall::Response &res){
    int function = req.function;
    switch(function){
    case INSERTIMAGE:
        ROS_INFO("a");
        break;
    case SHOWIMAGE:
        ROS_INFO("b");
        break;
    case HIDEIMAGE:
        ROS_INFO("c");
        break;
    case DELETEIMAGE:
        ROS_INFO("d");
        break;

    case SHOWMOBOT:
        ROS_INFO("e");
        break;
    case HIDEMOBOT:
        ROS_INFO("f");
        break;
    case DELETEMOBOT:
        ROS_INFO("g");
        break;

    case SHOWSESSION:
        ROS_INFO("h");
        break;
    case HIDESESSION:
        ROS_INFO("i");
        break;
    case DELETESESSION:
        ROS_INFO("j");
        break;

    case DELETEALLIMAGES:
        ROS_INFO("k");
        break;

    case SETIMAGEPOSE:
        ROS_INFO("l");
        break;

    case DELETEMOBOTMODEL:
        ROS_INFO("m");
        break;
    case DELETEALLMOBOTMODELS:
        ROS_INFO("n");
        break;
    case SETMOBOTMODEL:
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
    if(argv[0][0] == 'a'){
        updateInfoTest();
        return 1;
    }
    ros::spin();
    return 0;
}
