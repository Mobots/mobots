#ifndef MOBOTS_SLAM_H
#define MOBOTS_SLAM_H

#include "ros/ros.h"
#include "mobots_msgs/FeatureSetWithDeltaPoseAndID.h"
#include <treeoptimizer2.hh>
#include <vector>

/**
 * \class Slam
 * \brief ROS-Interface zum SLAM-System
 *
 * Implementiert einen Node und stellt somit die Schnittstelle der SLAM-Systems zum ROS-Framework dar.
 */
class Slam {
public:
    /**
     * @brief Constructor
     */
    Slam();

    /**
     * \brief Enable this display
     * @param force If false, does not re-enable if this display is already enabled.  If true, it does.
     */
    void enable(bool force = false);

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber1_;
    ros::Subscriber subscriber2_;
    ros::Subscriber subscriber3_;
    ros::Publisher publisher_;
    AISNavigation::TreeOptimizer2 pose_graph_;
    
    void callback1(const mobots_msgs::FeatureSetWithDeltaPoseAndID::ConstPtr& msg);
    void callback2(const mobots_msgs::FeatureSetWithDeltaPoseAndID::ConstPtr& msg);
    void callback3(const mobots_msgs::FeatureSetWithDeltaPoseAndID::ConstPtr& msg);
    void callback(const mobots_msgs::FeatureSetWithDeltaPoseAndID::ConstPtr& msg, uint mobot_id);
    
    static const uint MOBOT_COUNT = 3;
    int last_vertex[MOBOT_COUNT];
    
    std::vector<mobots_msgs::FeatureSet> feature_sets_;
};

#endif
