#ifndef IMAGE_MAP_WAYPOINT_H
#define IMAGE_MAP_WAYPOINT_H

#include <math.h>
#define PI 3.14159265

#include <QObject>
#include <QThread>
#include <QDebug>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mobots_msgs/PoseAndID.h>
#include <mobots_msgs/IDKeyValue.h>
#include <map_visualization/RemoteProcedureCall.h>

namespace map_visualization{

struct poseT{
    float x;
    float y;
    float theta;
};

class ImageMapWaypoint : public QObject{
    Q_OBJECT

public:
    ImageMapWaypoint(int argc, char** argv);
    ~ImageMapWaypoint();
    void updateInfoHandler(const mobots_msgs::IDKeyValue::ConstPtr& msg);
    void poseRelayHandler(const geometry_msgs::PoseStamped::ConstPtr& msgIn);
    void mobotPoseHandler(int mobotID, const geometry_msgs::Pose2D::ConstPtr &msg);
public Q_SLOTS:
    void process();
    void setActiveMobot(QString mobotID);
    int updateRviz(int sessionID, int mobotID, int key, int value);
Q_SIGNALS:
    void finished();
    void rvizChanged(int sessionID, int mobotID, int key, int value);
private:
    void unsubscribe();

    int init_argc;
    char** init_argv;
    int activeMobotID;
    int activeSessionID;
    int mobotPoseCount;
    std::vector<poseT> mobotPoseBuffer;
    ros::Subscriber* poseRelaySub;
    ros::Subscriber* updateInfoSub;
    ros::Publisher* poseRelayPub;
    ros::ServiceClient* updateRvizClient;
    std::vector<ros::Subscriber> mobotPoseSub;
    ros::NodeHandle *nh;
};

}

#endif // IMAGE_MAP_WAYPOINT_H
