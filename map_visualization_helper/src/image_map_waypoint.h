#ifndef IMAGE_MAP_WAYPOINT_H
#define IMAGE_MAP_WAYPOINT_H

#include <QObject>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mobots_msgs/IDKeyValue.h>
#include <map_visualization/RemoteProcedureCall.h>

namespace map_visualization{

class ImageMapWaypoint : public QObject
{
    Q_OBJECT
public:
    explicit ImageMapWaypoint(QObject *parent = 0);
    
};

}

#endif // IMAGE_MAP_WAYPOINT_H
