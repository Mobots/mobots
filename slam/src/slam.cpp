#include "slam.h"
#include "std_msgs/String.h"


Slam::Slam() :
  node_handle_(),
  subscriber1_(node_handle_.subscribe("/mobot1/FeatureSetWithDeltaPoseAndID", 1000, &Slam::callback1, this)),
  subscriber2_(node_handle_.subscribe("/mobot2/FeatureSetWithDeltaPoseAndID", 1000, &Slam::callback2, this)),
  subscriber3_(node_handle_.subscribe("/mobot3/FeatureSetWithDeltaPoseAndID", 1000, &Slam::callback3, this)),
  //publisher_(node_handle_.advertise<mobots_msgs::AbsoluteImagePoses>("AbsoluteImagePoses", 1000))
  pose_graph_()
{
  pose_graph_.initializeTreeParameters();
  pose_graph_.initializeOnlineOptimization();
}

void Slam::callback1(const mobots_msgs::FeatureSetWithDeltaPoseAndID::ConstPtr& msg)
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

void Slam::callback(const mobots_msgs::FeatureSetWithDeltaPoseAndID::ConstPtr& msg, uint mobot_id)
{
  ROS_INFO("Slam got a FeatureSetWithDeltaPoseAndID from mobot%u!", mobot_id);
  1. FeatureSet in Map unter Key (concatenated) ID abspeichern.
  2. last_id und current_id aktualisieren
  3. Neuen Vertex mit concatenated ID in TORO-Graph einfügen.
  4. Eventuell standardmäßig mit letztem FeatureSet matchen und Warnung an Moritz raushauen.
  5. FeatureSets finden, die sich zu matchen lohnen.
     Dazu TORO-Graph durchiterieren und Radien checken. Jeder mit jedem oder nur aktueller mit jedem?
  6. TORO-Algoritmus keine, eine oder mehrere Iterationen laufen lassen.
}

  7. public getter für aktuelle mobot_id
  8. public getter für aktuelle pose einer bestimmten id
  9. setter für start position

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "slam");

  Slam slam_instance = Slam();

  ros::spin();

  return 0;
}
