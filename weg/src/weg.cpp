#include "weg.h"






class Weg{
  
  
Weg weg(int mobotID):this.mobotID(mobotID) 			//Konstruktor Weg
{ 
  argc = 0;
  std::stringstream s;
  s << "weg_" << mobot_ID;
  ros::init(argc, (char**)argv, s.str());
  ros::NodeHandle nh;
  startWeg();
}
  
Weg ~weg()				//Destruktor
{
}
  
void Weg::startWeg()
{
  ROS_INFO("Mobot %d: Weg angeben",mobotID);
  nextPose_sub = nh.subscribe("/gui/pose", 30, &Weg::poseCallback, this);
  
  ros::spin();
}
  
void Weg::poseCallback(const mobots_msg::Pose2DPrio &next_pose)
{
  next = { next_pose.x, next_pose.y, next_pose.theta}
  if (next_pose.prio == 0) {
    list::clear();
    list::push_front(next);
  } else
  if (next_pose.prio == 1) {
    list::push_back(next);
  } else
  if (next_pose.prio > 1) {
    if (list.size() >= next_pose.prio) {
      list::insert(next_pose.prio, next);
    } else  
      list:: push_back(next);
  }
    
    
  
  
  
  
  
}
  
  
  
}
