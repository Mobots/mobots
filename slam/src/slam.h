#ifndef MOBOTS_SLAM_H
#define MOBOTS_SLAM_H

#include "ros/ros.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"
#include <treeoptimizer2.hh>
#include <feature_detector/FeaturesMatcher.h>

struct classcomp {
  bool operator() (const mobots_msgs::ID& lhs, const mobots_msgs::ID& rhs) const
  {
    if (lhs.session_id != rhs.session_id)
      return lhs.session_id < rhs.session_id;
    
    if (lhs.mobot_id != rhs.mobot_id)
      return lhs.mobot_id < rhs.mobot_id;
    
    return lhs.image_id < rhs.image_id;
  }
};

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

    std::map<uint32_t, FeatureSet> feature_sets_;
    CpuFeaturesMatcher features_matcher_;

    static const uint MOBOT_COUNT = 3;
    uint32_t last_id_[MOBOT_COUNT];
    uint32_t current_id_[MOBOT_COUNT];

    void callback1(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg);
    void callback2(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg);
    void callback3(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg);
    void callback(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg, uint mobot_id);

    uint32_t merge(mobots_msgs::ID const &id);
    mobots_msgs::ID split(uint32_t id);
    AISNavigation::TreeOptimizer2::Transformation convert(geometry_msgs::Pose2D pose);
};

#endif
