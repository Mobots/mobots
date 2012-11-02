#include <stdio.h>

#include <ros/ros.h>
#include <mobots_msgs/PoseAndID.h>
#include <mobots_msgs/IDKeyValue.h>
#include <map_visualization/RemoteProcedureCall.h>
#include <map_visualization/definitions.h>

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
    mobots_msgs::IDKeyValue msg;
    msg.id.session_id = 0;
    msg.id.mobot_id = 0;
    msg.id.image_id = 0;
    cout << "Enter the session:" << endl;
    cin >> msg.id.session_id;
    cout << "Enter the value:" << endl;
    cin >> msg.id;
    while(ros::ok){
        msg.id.mobot_id %= 3;
        cout << "Enter the key:" << endl;
        cin >> msg.key;
        cout << "Enter the value:" << endl;
        cin >> msg.value;
        updateInfoTestPub.publish(msg);
        ros::spinOnce();
        msg.id.mobot_id++;
    }
    return 1;
}


int updateRvizTest(){
    map_visualization::RemoteProcedureCall srv;
    int function = srv.request.function;
    switch(function){
    case insertImage:
        ROS_INFO("a");
        break;
    case showImage:
        ROS_INFO("b");
        break;
    case hideImage:
        ROS_INFO("c");
        break;
    case deleteImage:
        ROS_INFO("d");
        break;

    case showMobot:
        ROS_INFO("e");
        break;
    case hideMobot:
        ROS_INFO("f");
        break;
    case deleteMobot:
        ROS_INFO("g");
        break;

    case showSession:
        ROS_INFO("h");
        break;
    case hideSession:
        ROS_INFO("i");
        break;
    case deleteSession:
        ROS_INFO("j");
        break;

    case deleteAllImages:
        ROS_INFO("k");
        break;

    case setImagePose:
        ROS_INFO("l");
        break;

    case deleteMobotModel:
        ROS_INFO("m");
        break;
    case deleteAllMobotModels:
        ROS_INFO("n");
        break;
    case setMobotModel:
        ROS_INFO("o");
        break;
    }

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
        poseRelayTestHandler();
        return 1;
    }
    if(argv[1][0] == 'b'){
        updateInfoTest();
        return 1;
    }
    if(argv[1][0] == 'c'){
        updateRvizTest();
        return 1;
    }
    if(argv[1][0] == 'd'){
        poseRelayTestHandler();
        updateInfoTest();
        updateRvizTest();
        return 1;
    }
    ROS_INFO("usage: 'a' for poseRelayTest, 'b' for updateInfo, 'c' for updateRviz, 'd' for all");
}
