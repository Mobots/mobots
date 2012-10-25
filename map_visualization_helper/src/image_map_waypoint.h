#ifndef IMAGE_MAP_WAYPOINT_H
#define IMAGE_MAP_WAYPOINT_H

#include <QObject>
#include <QThread>
#include <QDebug>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mobots_msgs/IDKeyValue.h>
#include <map_visualization/RemoteProcedureCall.h>

namespace map_visualization{

class ImageMapWaypoint : public QThread
{
    Q_OBJECT
protected:
    void run();
};

}

#endif // IMAGE_MAP_WAYPOINT_H
