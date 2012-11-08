#define _USE_MATH_DEFINES

#include "slam.h"
#include "std_msgs/String.h"
#include <boost/foreach.hpp>
#include <cmath>
#include <feature_detector/MessageBridge.h>
#include "mobots_msgs/PoseAndID.h"
#include <boost/lexical_cast.hpp>

#define MY_INFO_STREAM(args) ROS_INFO_STREAM(__func__ << ": " << args)


/* Erlaube ich mir, weil darunter sowieso noch der Namespace TreeOptimizer2 liegt. */
using namespace AISNavigation;

static const double matching_varianz_translation = std::pow(10.0 / 2 * mobots_common::constants::image_height_in_meters / mobots_common::constants::image_height_in_pixels, 2); // Schätzung: 2-fache Standardabweichung = 10 Pixel
static const double matching_varianz_rotation = std::pow(10.0 / 2 * M_PI / 180, 2); // Schätzung: 2-fache Standardabweichung = 10 Grad
static const TreeOptimizer2::InformationMatrix matching_covarianz_matrix =
{
  {
    {matching_varianz_translation, 0, 0},
    {0, matching_varianz_translation, 0},
    {0, 0, matching_varianz_rotation}
  }
};

static const double mouse_varianz_translation = std::pow(0.005/2, 2); // Schätzung: 2-fache Standardabweichung = 5 mm
static const double mouse_varianz_rotation = std::pow(5.0/2 * M_PI/180, 2); // Schätzung: 2-fache Standardabweichung = 5 Grad
static const TreeOptimizer2::InformationMatrix mouse_covarianz_matrix =
{
  {
    {mouse_varianz_translation, 0, 0},
    {0, mouse_varianz_translation, 0},
    {0, 0, mouse_varianz_rotation}
  }
};
    

Slam::Slam() :
  node_handle_(),
  publisher_(node_handle_.advertise<mobots_msgs::PoseAndID>("slam/abs_pose", 1000)),
  pose_graph_(),
  feature_sets_(),
  features_matcher_(CpuFeaturesMatcher::ORB_DEFAULT),
  edge_states_()
{
  pose_graph_.verboseLevel = 0;

  /* Add vertex for mobot0, image0 at (0,0,0) */
  pose_graph_.addVertex(0, AISNavigation::TreeOptimizer2::Pose());

  for (int bot = 0; bot < mobots_common::constants::mobot_count; bot++)
  {
    if (bot > 0)
    {
      /* Add vertices for other mobots relative to mobot0*/
      mobots_msgs::ID bot_start_id;
      bot_start_id.session_id = 0;
      bot_start_id.mobot_id = bot;
      bot_start_id.image_id = 0;
      pose_graph_.addIncrementalEdge(0, merge(bot_start_id), start_pose_[bot], mouse_covarianz_matrix);
    }

    last_id_[bot] = NOT_RECEIVED_YET;
    current_id_[bot] = NOT_RECEIVED_YET;

    boost::function<void (const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg) > callback_function =
      boost::bind(&Slam::callback, this, _1, bot);
    subscriber_[bot] = node_handle_.subscribe("/mobot" + boost::lexical_cast<std::string > (bot) + "/featureset_pose_id", 1000, callback_function);
  }
}

void Slam::callback(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg, uint bot)
{
  MY_INFO_STREAM("Slam got a FeatureSetWithPoseAndID from mobot" << bot << '!');

  /* last_id_ und current_id_ aktualisieren */
  last_id_[bot] = current_id_[bot];
  current_id_[bot] = merge(msg->id);

  /* FeatureSet in Map unter Key (concatenated) ID abspeichern */
  MessageBridge::copyToCvStruct(msg->features, feature_sets_[current_id_[bot]]);

  addNewVertexToGraph(msg, bot);

  findEdgesBruteforce();

  runToro();

  publishOptimizedPoses();
}

void Slam::addNewVertexToGraph(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg, uint bot)
{
  /* Wenn wir noch keinen letztes Bild emfangen haben, besteht der Vertex schon (Konstruktor)
   * und es gibt kein FeatureSet zum matchen. */
  if (last_id_[bot] == NOT_RECEIVED_YET)
  {
    MY_INFO_STREAM("vertex (" << bot << ",0) is already in graph.");
    return;
  }
    
  if (tryToMatch(last_id_[bot], current_id_[bot]) == MATCHING_IMPOSSIBLE)
  {
    addNewVertexFromMouseData(msg, bot);
  }

  MY_INFO_STREAM("map size: " << pose_graph_.vertices.size());
}

enum Slam::EdgeState Slam::tryToMatch(const uint32_t v, const uint32_t w)
{
  MY_INFO_STREAM("v = " << print(v) << ", w = " << print(w));
  
  /* Nur matchen, wenn noch nicht gematcht wurde. */
  EdgeStateMap::iterator edge_state = edge_states_.find(std::make_pair(v, w));
  if (edge_state != edge_states_.end())
  {
    MY_INFO_STREAM("Skipping combination because it was already tryed.");
    return edge_state->second;
  }
  

#if 0
  //Save position of current standard output
  fpos_t pos;
  fgetpos(stdout, &pos);
  int fd = dup(fileno(stdout));
  freopen("/dev/null", "w", stdout);
#endif
  
  /* Jetzt werfen wir Jonas sein Matcher an. */
  MatchResult delta;
  bool success = features_matcher_.match(feature_sets_[v], feature_sets_[w], delta);
  
#if 0
  //Flush stdout so any buffered messages are delivered
  fflush(stdout);
  //Close file and restore standard output to stdout - which should be the terminal
  dup2(fd, fileno(stdout));
  close(fd);
  clearerr(stdout);
  fsetpos(stdout, &pos);
#endif
  
  if (!success)
  {
    MY_INFO_STREAM("Failed to match!");
    return edge_states_[std::make_pair(v, w)] = MATCHING_IMPOSSIBLE;;
  }

  /* Matching-Ergebnis als Edge zwischen den zwei Vertices einfügen */

  TreeOptimizer2::Transformation t = convertPixelsToMeters(delta);

  TreeOptimizer2::Edge* result = pose_graph_.addIncrementalEdge(v, w, t, matching_covarianz_matrix);
  MY_INFO_STREAM("Added edge with x = " << t.translation().x() << ", y = " << t.translation().y() << ", theta = " << t.rotation() << ". Result: " << result);

  return edge_states_[std::make_pair(v, w)] = MATCHED;
}

void Slam::addNewVertexFromMouseData(const boost::shared_ptr<mobots_msgs::FeatureSetWithPoseAndID const>& msg, uint bot)
{
  /* DeltaPose als Edge zwischen den zwei Vertices einfügen */
  TreeOptimizer2::Transformation t = TreeOptimizer2::Transformation(msg->pose.x, msg->pose.y, msg->pose.theta);

  TreeOptimizer2::Edge* result = pose_graph_.addIncrementalEdge(last_id_[bot], current_id_[bot], t, mouse_covarianz_matrix);
  
  MY_INFO_STREAM("Added edge with x = " << t.translation().x() << ", y = " << t.translation().y() << ", theta = " << t.rotation() << ". Result: " << result);
}

void Slam::findEdgesBruteforce()
{
#if 0
  BOOST_FOREACH(TreeOptimizer2::VertexMap::value_type &v, pose_graph_.vertices)
  {
    tryToMatchWithAllOthers(v);
  }
#endif
}

void Slam::tryToMatchWithAllOthers(const TreeOptimizer2::VertexMap::value_type &v, bool distance_check)
{
#if 0
  BOOST_FOREACH(TreeOptimizer2::VertexMap::value_type &other, pose_graph_.vertices)
  {
    /* Jede Kombination nur einmal bilden. */
    if (&v == &other)
      break;
    
    tryToMatch(v.first, w.first);

  }
#endif
}

void Slam::runToro()
{
  /* Lass den TORO laufen! */
  //pose_graph_.buildSimpleTree();
  pose_graph_.initializeTreeParameters();
  pose_graph_.initializeOptimization(); // Es gibt noch eine Online-Variante von der Methode...
  for (uint i = 0; i < ITERATIONS_PER_NEW_IMAGE; ++i)
  {
    pose_graph_.iterate();
    MY_INFO_STREAM("Iterated once. Error: " << pose_graph_.error());
  }
}

void Slam::publishOptimizedPoses()
{
  /* (Hoffentlich) optimierte Image-Poses rausschicken. */
  BOOST_FOREACH(TreeOptimizer2::VertexMap::value_type &v, pose_graph_.vertices)
  {
    mobots_msgs::PoseAndID pai;
    pai.id = split(v.first);
    
    if(current_id_[pai.id.mobot_id] == NOT_RECEIVED_YET)
      continue;
    
    pai.pose = convert(v.second->pose);
    publisher_.publish(pai);
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

std::string Slam::print(uint32_t id)
{
  mobots_msgs::ID id_msg = split(id);
  return '(' + boost::lexical_cast<std::string>(id_msg.mobot_id) + ',' + boost::lexical_cast<std::string>(id_msg.image_id) + ')';
}

TreeOptimizer2::Transformation Slam::convertPixelsToMeters(const MatchResult& result)
{
  float x_in_meters = result.delta.x * mobots_common::constants::image_height_in_meters / mobots_common::constants::image_width_in_pixels;
  float y_in_meters = -result.delta.y * mobots_common::constants::image_height_in_meters / mobots_common::constants::image_height_in_pixels;
  assert(0 <= result.delta.theta && result.delta.theta < 2 * M_PI);
  return TreeOptimizer2::Transformation(x_in_meters, y_in_meters, result.delta.theta);
}

geometry_msgs::Pose2D Slam::convert(const TreeOptimizer2::Pose& toro_pose)
{
  geometry_msgs::Pose2D ros_pose;
  ros_pose.x = toro_pose.x();
  ros_pose.y = toro_pose.y();
  ros_pose.theta = toro_pose.theta();
  return ros_pose;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "slam");

  Slam slam_instance = Slam();

  MY_INFO_STREAM("Slam is spinning now!");

  ros::spin();

  return 0;
}
