#ifndef IMAGE_MAP_WAYPOINT_H
#define IMAGE_MAP_WAYPOINT_H

#include <QObject>
#include <QThread>
#include <QDebug>

#include <boost/bind.hpp>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <mobots_msgs/IDKeyValue.h>
#include <map_visualization/RemoteProcedureCall.h>

namespace map_visualization{

class ImageMapWaypoint : public QThread
{
    Q_OBJECT
public:
    ImageMapWaypoint(int argc, char** argv);
    ~ImageMapWaypoint();
    bool init();
protected:
    void run();
public Q_SLOTS:
    void setActiveMobot(int mobotID);
    void poseRelayHandler(const geometry_msgs::PoseStamped::ConstPtr& msg);
private:
    int init_argc;
    char** init_argv;
    int activeMobotID;
    ros::Subscriber poseRouterSub;
    ros::Publisher poseRouterPub;
};

}

#endif // IMAGE_MAP_WAYPOINT_H
