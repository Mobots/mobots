#ifndef MOBOTS_SLAM_H
#define MOBOTS_SLAM_H

#include "ros/ros.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"
#include <treeoptimizer2.hh>
#include <feature_detector/FeaturesMatcher.h>

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
//    void enable(bool force = false);

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber1_;
    ros::Subscriber subscriber2_;
    ros::Subscriber subscriber3_;
    ros::Publisher publisher_;

    AISNavigation::TreeOptimizer2 pose_graph_;
    std::map<uint32_t, mobots_msgs::FeatureSet> feature_sets_;
    CpuFeaturesMatcher features_matcher_;

    static const uint MOBOT_COUNT = 3;
    int last_id_[MOBOT_COUNT];
    int current_id_[MOBOT_COUNT];

    void callback1(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg);
    void callback2(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg);
    void callback3(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg);
    void callback(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg, uint mobot_id);

    uint32_t concatenate(mobots_msgs::ID const &id);
};

#endif
