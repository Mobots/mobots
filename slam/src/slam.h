#ifndef MOBOTS_SLAM_H
#define MOBOTS_SLAM_H

#include "ros/ros.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"
#include <treeoptimizer2.hh>
#include <feature_detector/FeaturesMatcher.h>
#include <utility>
#include <mobots_common/constants.h>


/**
 * \class Slam
 * \brief ROS-Interface zum SLAM-System
 *
 * Implementiert einen Node und stellt somit die Schnittstelle der SLAM-Systems zum ROS-Framework dar.
 */
class Slam {

public:
    Slam();

private:
/* Constants */
    static const uint ITERATIONS_PER_NEW_IMAGE = 7;
    enum EdgeState {MATCHING_IMPOSSIBLE = 0, MATCHED};
    static const uint32_t NOT_RECEIVED_YET = 0xffffffff;
    
/*    static const float matching_varianz_translation;
    static const float matching_varianz_rotation;
    static const AISNavigation::TreeOptimizer2::InformationMatrix matching_covarianz_matrix;
    
    static const float mouse_varianz_translation;
    static const float mouse_varianz_rotation;
    static const AISNavigation::TreeOptimizer2::InformationMatrix mouse_covarianz_matrix;
*/
    
/* Member variables */
    uint32_t last_id_[mobots_common::constants::mobot_count];
    uint32_t current_id_[mobots_common::constants::mobot_count];
    AISNavigation::TreeOptimizer2::Transformation start_pose_[mobots_common::constants::mobot_count];

    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber_[mobots_common::constants::mobot_count];
    ros::Publisher publisher_;

    AISNavigation::TreeOptimizer2 pose_graph_;

    std::map<uint32_t, FeatureSet> feature_sets_;
    CpuFeaturesMatcher features_matcher_;
    
    typedef std::map< std::pair<uint32_t,uint32_t> , EdgeState > EdgeStateMap;
    EdgeStateMap edge_states_; 

/* Member functions */
    void callback(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg, uint mobot_id);
    void addNewVertexToGraph(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg, uint bot);
    enum Slam::EdgeState tryToMatch(const uint32_t v, const uint32_t w);
    void addNewVertexFromMouseData(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg, uint bot);
    
    void findEdgesBruteforce(bool distance_check = true);
    void tryToMatchWithAllOthers(const AISNavigation::TreeOptimizer2::VertexMap::value_type &v, bool distance_check = true);

    void runToro();
    void publishOptimizedPoses();
    
/* Helper functions */
    uint32_t merge(mobots_msgs::ID const &id);
    mobots_msgs::ID split(uint32_t id);
    std::string print(uint32_t id);
    
    AISNavigation::TreeOptimizer2::Transformation convertPixelsToMeters(const MatchResult& result);
    geometry_msgs::Pose2D convert(const AISNavigation::TreeOptimizer2::Pose& toro_pose);
};

#endif
