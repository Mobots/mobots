#include "slam.h"
#include "std_msgs/String.h"
#include <boost/foreach.hpp>
#include <math.h>
#include <feature_detector/MessageBridge.h>

 /* Erlaube ich mir, weil darunter sowieso noch der Namespace TreeOptimizer2 liegt. */
using namespace AISNavigation;

Slam::Slam() :
  node_handle_(),
  subscriber1_(node_handle_.subscribe("/mobot1/featureset_pose_id", 1000, &Slam::callback1, this)),
  subscriber2_(node_handle_.subscribe("/mobot2/featureset_pose_id", 1000, &Slam::callback2, this)),
  subscriber3_(node_handle_.subscribe("/mobot3/featureset_pose_id", 1000, &Slam::callback3, this)),
  //publisher_(node_handle_.advertise<mobots_msgs::AbsoluteImagePoses>("AbsoluteImagePoses", 1000)),
  pose_graph_(),
  features_matcher_(CpuFeaturesMatcher::ORB_DEFAULT)
{
  //pose_graph_.initializeTreeParameters(); //Use of uninitialised value of size 8    ==14818==    at 0x42A363: AISNavigation::ParameterPropagator::perform(AISNavigation::TreePoseGraph<Operations2D<double> >::Vertex*) (treeoptimizer2.cpp:59)
  //pose_graph_.initializeOnlineOptimization(); //Conditional jump or move depends on uninitialised value(s)

  for(uint bot = 1; bot <= 1; ++bot) //TODO: 1 durch MOBOT_COUNT ersetzten
  {
    last_id_[bot] = 0;
    
    mobots_msgs::ID id;
    id.session_id = 0;
    id.mobot_id = bot;
    id.image_id = 0;
    current_id_[bot] = merge(id);
    
    TreeOptimizer2::Pose initial_pose = TreeOptimizer2::Pose(0, 0, 0);
    pose_graph_.addVertex(current_id_[bot], initial_pose);
  }
}

void Slam::callback1(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg)
{
  callback(msg, 1);
}

void Slam::callback2(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg)
{
  callback(msg, 2);
}

void Slam::callback3(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg)
{
  callback(msg, 3);
}

void Slam::callback(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg, uint bot)
{
  ROS_INFO("Slam got a FeatureSetWithPoseAndID from mobot%u!", bot);

  /* last_id_ und current_id_ aktualisieren */
  last_id_[bot] = current_id_[bot];
  current_id_[bot] = merge(msg->id);
  
  /* FeatureSet in Map unter Key (concatenated) ID abspeichern */
  FeatureSet bla;
  MessageBridge::copyToCvStruct( msg->features, bla );
  feature_sets_[current_id_[bot]] = bla;

  /* Neue current_pose auf Basis von last_pose und DeltaPose schätzen */
  TreeOptimizer2::Pose last_pose = pose_graph_.vertex(last_id_[bot])->pose;
  TreeOptimizer2::Pose current_pose = TreeOptimizer2::Pose(last_pose.x() + msg->pose.x, last_pose.y() + msg->pose.y, last_pose.theta() + msg->pose.theta);

  /* Neuen Vertex mit concatenated ID in TORO-Graph einfügen */
  pose_graph_.addVertex(current_id_[bot], current_pose);
  ROS_INFO_STREAM("map size: " << pose_graph_.vertices.size());

  
#if 0 // Zwischen zwei Vertices kann anscheinend nur eine Edge je Richtung gesetzt werden...
   /* DeltaPose als Edge zwischen den zwei Vertices einfügen */

  TreeOptimizer2::Transformation t
    = TreeOptimizer2::Transformation(msg->pose.x, msg->pose.y, msg->pose.theta);
  
  TreeOptimizer2::InformationMatrix m;
  m.values[0][0] = 1; m.values[0][1] = 0; m.values[0][2] = 0;
  m.values[1][0] = 0; m.values[1][1] = 1; m.values[1][2] = 0;
  m.values[2][0] = 0; m.values[2][1] = 0; m.values[2][2] = 1;
  
  pose_graph_.addEdge(pose_graph_.vertex(last_id_[bot]), pose_graph_.vertex(current_id_[bot]), t, m);
#endif
  
  /*
  4. Eventuell standardmäßig mit letztem FeatureSet matchen und Warnung an Moritz raushauen.
  5. FeatureSets finden, die sich zu matchen lohnen.
     Dazu TORO-Graph durchiterieren und Radien checken. Jeder mit jedem oder nur aktueller mit jedem?
  6. TORO-Algoritmus keine, eine oder mehrere Iterationen laufen lassen.
   */

  BOOST_FOREACH(TreeOptimizer2::VertexMap::value_type &v, pose_graph_.vertices)
  {
    BOOST_FOREACH(TreeOptimizer2::VertexMap::value_type &w, pose_graph_.vertices)
    {
      /* Jede Kombination nur einmal bilden. */
      if (&v == &w)
        break;
      
      /* Zum nullten Vertex gibt es kein FeatureSet. */
      if (split(v.first).image_id == 0 || split(w.first).image_id == 0)
        continue;
      
      /* Nur matchen, wenn die Bildmittelpunkte ausreichend nah beieinander liegen. */
      double norm = sqrt( TreeOptimizer2::Translation(w.second->pose.x() - v.second->pose.x(), w.second->pose.y() - v.second->pose.y()).norm2() );
      ROS_INFO_STREAM("Distance between image " << split(v.second->id).image_id << " and " << split(w.second->id).image_id << " is " << norm);
      if (norm > 480)
        continue;
      
      uint edge_count = 0;
      for ( TreeOptimizer2::EdgeList::iterator v_itr = v.second->edges.begin(); v_itr != v.second->edges.end(); ++v_itr )
      {
        for ( TreeOptimizer2::EdgeList::iterator w_itr = v.second->edges.begin(); w_itr != v.second->edges.end(); ++w_itr )
        {
          if ( v_itr == w_itr )
            ++edge_count;
          // hier is noch n bug
        }
      }
      ROS_INFO_STREAM("edge_count = " << edge_count);
      
      if (edge_count) {
        continue;
      }
      
      /*ROS_INFO_STREAM(feature_sets_[v.first].keyPoints.size() << "v" << feature_sets_[v.first].descriptors.rows);
      ROS_INFO_STREAM(feature_sets_[w.first].keyPoints.size() << "w" << feature_sets_[w.first].descriptors.rows);*/

      geometry_msgs::Pose2D delta;        
      if ( features_matcher_.match(feature_sets_[v.first], feature_sets_[w.first], delta) )
      {
        /* Matching-Ergebnis als Edge zwischen den zwei Vertices einfügen */
        TreeOptimizer2::Transformation t = convert(delta);

        ROS_INFO_STREAM("matching result: x = " << t.translation().x() << ", y = " << t.translation().y() << ", theta = " << t.rotation());

        TreeOptimizer2::InformationMatrix m;
        m.values[0][0] = 1; m.values[0][1] = 0; m.values[0][2] = 0;
        m.values[1][0] = 0; m.values[1][1] = 1; m.values[1][2] = 0;
        m.values[2][0] = 0; m.values[2][1] = 0; m.values[2][2] = 1;

        ROS_INFO_STREAM("huhu" << pose_graph_.addEdge(pose_graph_.vertex(last_id_[bot]), pose_graph_.vertex(current_id_[bot]), t, m) );

      }
    }
  }
}

uint32_t Slam::merge(mobots_msgs::ID const &id)
{
  return ((uint32_t) id.mobot_id) << 16 | id.image_id;
}

mobots_msgs::ID Slam::split(uint32_t id)
{
  mobots_msgs::ID result;
  result.session_id = -1;
  result.mobot_id = id >> 16;
  result.image_id = (uint16_t) id;
  return result;
}

TreeOptimizer2::Transformation Slam::convert(geometry_msgs::Pose2D pose)
{
  return TreeOptimizer2::Transformation(pose.x, pose.y, pose.theta);
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

  ROS_INFO("Slam is spinning now!");
  
  ros::spin();

  return 0;
}
