#include "image_map_waypoint.h"

namespace map_visualization{

ImageMapWaypoint::ImageMapWaypoint(QObject *parent) :
    QObject(parent)
{
    int argc = 0;
    char* argv[];
    argv = &"image_map_info";
    ros::init(argc, argv, "image_map_info");
}

}
