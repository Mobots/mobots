#include "slam.h"
#include "std_msgs/String.h"

Slam::Slam() :
  node_handle_(),
  subscriber1_(node_handle_.subscribe("/mobot1/FeatureSetWithDeltaPoseAndID", 1000, &Slam::callback1, this)),
  subscriber2_(node_handle_.subscribe("/mobot2/FeatureSetWithDeltaPoseAndID", 1000, &Slam::callback2, this)),
  subscriber3_(node_handle_.subscribe("/mobot3/FeatureSetWithDeltaPoseAndID", 1000, &Slam::callback3, this)),
  //publisher_(node_handle_.advertise<mobots_msgs::AbsoluteImagePoses>("AbsoluteImagePoses", 1000)),
  pose_graph_(),
  features_matcher_(CpuFeaturesMatcher::ORB_DEFAULT)
{
  pose_graph_.initializeTreeParameters();
  pose_graph_.initializeOnlineOptimization();
}

void Slam::callback1(const boost::shared_ptr<mobots_msgs::FeatureSetWithDeltaPoseAndID const>& msg)
{
  callback(msg, 1);
}

void Slam::callback2(const mobots_msgs::FeatureSetWithDeltaPoseAndID::ConstPtr& msg)
{
  callback(msg, 2);
}

void Slam::callback3(const mobots_msgs::FeatureSetWithDeltaPoseAndID::ConstPtr& msg)
{
  callback(msg, 3);
}

void Slam::callback(const boost::shared_ptr<mobots_msgs::FeatureSetWithDeltaPoseAndID const>& msg, uint bot)
{
  ROS_INFO("Slam got a FeatureSetWithDeltaPoseAndID from mobot%u!", bot);

  /* FeatureSet in Map unter Key (concatenated) ID abspeichern */
  feature_sets_[msg->id] = msg->features;

  /* last_id_ und current_id_ aktualisieren */
  last_id_[bot] = current_id_[bot];
  current_id_[bot] = concatenate(msg->id->image_id);

  /* Neue current_pose auf Basis von last_pose und DeltaPose schätzen */
  AISNavigation::TreeOptimizer2::Pose last_pose = pose_graph_.vertex(last_id_[bot])->pose;
  AISNavigation::TreeOptimizer2::Pose current_pose = new AISNavigation::TreeOptimizer2::Pose(last_pose.x() + msg->delta_pose->x, last_pose.y() + msg->delta_pose->y, last_pose.theta() + msg->delta_pose->theta);

  /* Neuen Vertex mit concatenated ID in TORO-Graph einfügen */
  pose_graph_.addVertex(current_id_[bot], current_pose);

  /* DeltaPose als Edge zwischen den zwei Vertices einfügen */

  AISNavigation::TreeOptimizer2::Transformation t
    = AISNavigation::TreeOptimizer2::Transformation(msg->delta_pose->x, msg->delta_pose->y, msg->delta_pose->theta);
  
  AISNavigation::TreeOptimizer2::InformationMatrix m;
  m.values[0] = {1, 0, 0};
  m.values[1] = {0, 1, 0};
  m.values[2] = {0, 0, 1};
  
  pose_graph_.addEdge(pose_graph_.vertex(last_id_), pose_graph_.vertex(current_id_), t, m);
  
  /*
  4. Eventuell standardmäßig mit letztem FeatureSet matchen und Warnung an Moritz raushauen.
  5. FeatureSets finden, die sich zu matchen lohnen.
     Dazu TORO-Graph durchiterieren und Radien checken. Jeder mit jedem oder nur aktueller mit jedem?
  6. TORO-Algoritmus keine, eine oder mehrere Iterationen laufen lassen.
   */

}

int Slam::concatenate(const mobots_msgs::ID::ConstPtr& id)
{
  return ((uint32_t) id->mobot_id) << 16 | id->image_id;
}

/*
  7. public getter für aktuelle mobot_id
  8. public getter für aktuelle pose einer bestimmten id
  9. setter für start position
 */

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "slam");

  Slam slam_instance = Slam();

  ros::spin();

  return 0;
}
