#include "image_map_waypoint.h"

namespace map_visualization{

void ImageMapWaypoint::run(){
    qDebug() << "hello from worker thread " << thread()->currentThreadId();
    int argc = 1;
    char* argv[4];
    *argv = new char[4];
    argv[0][0] = 'a';
    argv[0][0] = 'b';
    argv[0][0] = 'c';
    argv[0][0] = '\0';
    ros::init(argc, argv, "image_map_info");
    ROS_INFO("I'm alive");
}

}
