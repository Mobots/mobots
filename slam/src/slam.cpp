#define _USE_MATH_DEFINES

#include "slam.h"
#include "std_msgs/String.h"
#include <boost/foreach.hpp>
#include <cmath>
#include <feature_detector/MessageBridge.h>
#include "mobots_msgs/PoseAndID.h"
#include "mobots_common/constants.h"

 /* Erlaube ich mir, weil darunter sowieso noch der Namespace TreeOptimizer2 liegt. */
using namespace AISNavigation;

Slam::Slam() :
  node_handle_(),
  subscriber1_(node_handle_.subscribe("/mobot1/featureset_pose_id", 1000, &Slam::callback1, this)),
  subscriber2_(node_handle_.subscribe("/mobot2/featureset_pose_id", 1000, &Slam::callback2, this)),
  subscriber3_(node_handle_.subscribe("/mobot3/featureset_pose_id", 1000, &Slam::callback3, this)),
  publisher_(node_handle_.advertise<mobots_msgs::PoseAndID>("slam/abs_pose", 1000)),
  pose_graph_(),
  features_matcher_(CpuFeaturesMatcher::ORB_DEFAULT)
{
  pose_graph_.verboseLevel = 0;
  
  for(uint bot = 1; bot <= 1; ++bot) //TODO: 1 durch MOBOT_COUNT ersetzten
  {
    last_id_[bot] = 0;
    current_id_[bot] = 0;
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

  addNewVertexToGraph(msg, bot);

  findEdgesBruteforce();
  
  /* TODO: Nachfolgender Hack oder ähnliches wird auch nötig, wenn ein Bild reinkommt, das mit keinem gematcht wreden kann */
  for(uint bot = 0; bot < MOBOT_COUNT; ++bot)
  {
    /* TORO kann nicht laufen, wenn es Vertexe ohne Kanten gibt. */
    if (!last_id_[bot])
      return;
  }
  
  runToro();
  
  publishOptimizedPoses();

}

void Slam::addNewVertexToGraph(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg, uint bot)
{
  /* last_id_ und current_id_ aktualisieren */
  last_id_[bot] = current_id_[bot];
  current_id_[bot] = merge(msg->id);
  
  /* FeatureSet in Map unter Key (concatenated) ID abspeichern */
  MessageBridge::copyToCvStruct( msg->features, feature_sets_[current_id_[bot]] );

  TreeOptimizer2::Pose current_pose = TreeOptimizer2::Pose();
  if (last_id_[bot]) // Nur, wenn wir schon eine letzte Nachricht haben...
  {
    /* Neue current_pose auf Basis von last_pose und DeltaPose schätzen */
    TreeOptimizer2::Pose last_pose = pose_graph_.vertex(last_id_[bot])->pose;
    current_pose = TreeOptimizer2::Pose(last_pose.x() + msg->pose.x, last_pose.y() + msg->pose.y, last_pose.theta() + msg->pose.theta);
  }
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
}

void Slam::findEdgesBruteforce()
{
  BOOST_FOREACH(TreeOptimizer2::VertexMap::value_type &v, pose_graph_.vertices)
  {
    BOOST_FOREACH(TreeOptimizer2::VertexMap::value_type &w, pose_graph_.vertices)
    {
      /* Jede Kombination nur einmal bilden. */
      if (&v == &w)
        break;
      
      /* Nur matchen, wenn die Bildmittelpunkte ausreichend nah beieinander liegen. */
      double norm = sqrt( TreeOptimizer2::Translation(w.second->pose.x() - v.second->pose.x(), w.second->pose.y() - v.second->pose.y()).norm2() );
      ROS_INFO_STREAM("Distance between image " << split(v.second->id).image_id << " and " << split(w.second->id).image_id << " is " << norm);
      if (norm > 480) {
        ROS_INFO_STREAM("Skipping combination because of to big distance.");
        continue;
      }
      
      /* Nur matchen, wenn noch nicht gematcht wurde. */
      uint edge_count = 0;
      for ( TreeOptimizer2::EdgeList::iterator v_itr = v.second->edges.begin(); v_itr != v.second->edges.end(); ++v_itr )
      {
        for ( TreeOptimizer2::EdgeList::iterator w_itr = w.second->edges.begin(); w_itr != w.second->edges.end(); ++w_itr )
        {
          if ( *v_itr == *w_itr )
            ++edge_count;
        }
      }
      if (edge_count) {
        ROS_INFO_STREAM("Skipping combination because it has already an edge.");
        continue;
      }
      
      /* Jetzt werfen wir Jonas sein Matcher an. */
      geometry_msgs::Pose2D delta;        
      if ( ! features_matcher_.match(feature_sets_[v.first], feature_sets_[w.first], delta) )
      {
        ROS_INFO_STREAM("Skippung combination because of matching error.");
        continue;
      }
      
      /* Matching-Ergebnis als Edge zwischen den zwei Vertices einfügen */
      
      TreeOptimizer2::Transformation t = convertPixelsToMeters(delta);

      // Lustige Covarianzmatrix erstellen
      float varianz_translation = std::pow(0.5/2 * mobots_common::constants::image_height_in_meters / mobots_common::constants::image_height_in_meters, 2); // Schätzung: 2-fache Standardabweichung 10 Pixel
      float varianz_rotation = std::pow(0.5/2 * M_PI/180, 2); // Schätzung: 2-fache Standardabweichung 10 Grad
      
      TreeOptimizer2::InformationMatrix m;
      m.values[0][0] = varianz_translation; m.values[0][1] = 0;                   m.values[0][2] = 0;
      m.values[1][0] = 0;                   m.values[1][1] = varianz_translation; m.values[1][2] = 0;
      m.values[2][0] = 0;                   m.values[2][1] = 0;                   m.values[2][2] = varianz_rotation;

      ROS_INFO_STREAM("Adding edge with x = " << t.translation().x() << ", y = " << t.translation().y() << ", theta = " << t.rotation() << ". Result: " << pose_graph_.addEdge(v.second, w.second, t, m) );
    }
  }
}

void Slam::runToro()
{
  /* Lass den TORO laufen! */
  pose_graph_.buildSimpleTree();
  pose_graph_.initializeTreeParameters();
  pose_graph_.initializeOptimization(); // Es gibt noch eine Online-Variante von der Methode...
  for(uint i = 0; i < ITERATIONS_PER_NEW_IMAGE; ++i)
  {
    pose_graph_.iterate();
    ROS_INFO_STREAM("Iterated once. Error: " << pose_graph_.error() );
  }
}

void Slam::publishOptimizedPoses()
{
  int i = 0;
  /* (Hoffentlich) optimierte Image-Poses rausschicken. */
  BOOST_FOREACH(TreeOptimizer2::VertexMap::value_type &v, pose_graph_.vertices)
  {
    mobots_msgs::PoseAndID pai;
    pai.id = split(v.first);
    pai.pose = convert(v.second->pose);
    /*pai.pose.x = i * mobots_common::constants::image_width_in_meters;
    pai.pose.y = i * mobots_common::constants::image_height_in_meters;
    pai.pose.theta = i*M_PI/8;*/
    publisher_.publish(pai);
    i++;
  }
}

uint32_t Slam::merge(mobots_msgs::ID const &id)
{
  return ((uint32_t) id.mobot_id) << 16 | id.image_id;
}

mobots_msgs::ID Slam::split(uint32_t id)
{
  mobots_msgs::ID result;
  result.session_id = 0;
  result.mobot_id = id >> 16;
  result.image_id = (uint16_t) id;
  return result;
}

TreeOptimizer2::Transformation Slam::convertPixelsToMeters(geometry_msgs::Pose2D pose)
{
  float x_in_meters = pose.x * mobots_common::constants::image_height_in_meters / mobots_common::constants::image_width_in_pixels;
  float y_in_meters = - pose.y * mobots_common::constants::image_height_in_meters / mobots_common::constants::image_height_in_pixels;
  assert(0 <= pose.theta && pose.theta < 2*M_PI);
  return TreeOptimizer2::Transformation(x_in_meters, y_in_meters, - pose.theta);
}

geometry_msgs::Pose2D Slam::convert(TreeOptimizer2::Pose toro_pose)
{
  geometry_msgs::Pose2D ros_pose;
  ros_pose.x = toro_pose.x();
  ros_pose.y = toro_pose.y();
  ros_pose.theta = toro_pose.theta();
  return ros_pose;
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
