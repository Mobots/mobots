#include "weg.h"


class Weg{
  
Weg weg(int mobotID, double bParam):this.mobotID(mobotID),this.bParam(bParam) 			//Konstruktor Weg
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
  nextPose_sub = nh.subscribe("mobot_pose/waypoint", 30, &Weg::poseCallback, this);
  pose2D_pub = nh.advertise<geometry_msgs::Pose2D>("mobot_pose/globalPose", 5);
  sollV_pub = nh.advertise<geometry_msgs::Pose2D>("driver/sollV", 2);
  
  //Services
  ros::ServiceClient = nh.serviceClient<shutter::delta>("mobot_pose/getDelta");
  ros::ServiceCenter service = nh.advertiseService("mobot_pose/changeGlobalPose", changeGlobalPose);
  
  //Parameter
  ros::param::param<double>("sBrems",sBrems,0.2);
  ros::param::param<double>("rootParam",rootParam,2.0);
  ros::param::param<double>("vFac",vFac,1);  //TODO
//vFac ist der zusammenhang: Vmaximal/1000 zwischen promilledaten und realität
  ros::param::param<double>("rad",rad,0.14); //TODO
  ros::param::param<double>("vMax",vMax,800);
  ros::param::param<double>("minS",minS,0.02);
  ros::param::param<double>("minDegree",minDegree,1); //TODO
  
  
  bParam=pow(1000*vFac,rootParam)/sBrems;  
  
  ros::spin();
}
  
void Weg::poseCallback(const mobots_msgs::Pose2DPrio &next_pose)
{
  listManage({ next_pose.x, next_pose.y, next_pose.theta}, next_pose.prio);
}
  
  
  
void Weg::listManage(pose next, int prio)
{
  bool flag = false;
  switch(prio)
  {
    case -2:
      list::pop_front();
      flag = true;
    case -1:
      list::clear();
      list::push_front(globalPose);
      flag = true;
    break;
    case 0:
      list::push_front(next);
      flag = true;
    break;
    case 1:
      list::push_back(next);
      flag = true;
    break;
    default:
    if (list.size() > prio) {
      list::insert(prio, next);
    } else { 
      list::push_back(next);
      if (list.size() < 2) {     flag = true; }
    }
  }
  
  if (flag)  {
    //Fall: Nr0 oder Nr1 aktualisiert, Nr1 wegen Bremsvermeidung.
    //irgendwas aktualisieren
    sollS = list::front();
  }  
}
  
  
bool weg::changeGlobalPose(weg::poseChange::Request &req, weg::poseChange::Response &res)
{
  shutter::delta srv;
  if (client.call(srv))
  {						//LOCK ???
   globalPose.x=srv.response.x+req.x;
   globalPose.y=srv.response.y+req.y;
   globalPose.theta=srv.response.theta+req.theta;
  } else {
    ROS_ERROR("Failed to call getDelta");
    return false;
  }
  return true;
  
}



void Weg::mouseCallback(const geometry_msgs::Pose2D &mouse_data) {
  globalPose.x+= mouse_data.x;
  globalPose.y+= mouse_data.y;
  globalPose.theta += mouse_data.theta;
  geometry_msgs::Pose2D pub_pose;
  pub_pose.x = globalPose.x;
  pub_pose.y = globalPose.y;
  pub_pose.theta = globalPose.theta;
  pose2D_pub.publish(pub_pose);
  
  regel();
}
    
    //Annahme: Strecken in m
    
void Weg::regel()
{
 double eX =  sollS.x - globalPose.x;
 double eY =  sollS.y - globalPose.y;
 double eTheta =  sollS.theta - globalPose.theta;
 if (eX < minS && eY < minS && eTheta < minDegree*(M_Pi / 360)) { //Ziel erreicht
   listManage(NULL,-2);
 }
 if (wayType == STIFF) || (wayType == FAST) //TODO
 {
   sollV.x=regelFkt(sollS.x);
   sollV.y=regelFkt(sollS.y);
   sollV.theta=regelFktDreh(sollS.theta*rad); //Weitergabe Regeldifferenz als Strecke auf Kreisbahn
   
   
   
   
   
   
 }
  
  
  
}



/*
 * Die Regelfunktion zwischen Strecke u. Geschwindigkeit stellt eine Wurzelfunktion dar.
 * Der Exponent wird mittels "rootParam" festgelegt. bParam skaliert auf den 
 * gewünschten Bremsweg hoch, dieser kann durch bmax begrenzt werden.
 * V wird metrisch verrechnet und dann mittels vFac auf Promille skaliert.
 * 
 */
double Weg::regelFkt(double e)
{
 if (e>0) {
   double d = pow(e*bParam, 1.0/rootParam);
   d=d/vParam;
   return (d>vMax) ? vMax :d;
 }
 if (e<0) {
   double d =-(pow(-e*bParam, 1.0/rootParam));
   return (d<-vMax) ? -vMax :d;
 }
  
}
  
  /*
   * Linear steigende Regelfunktion, Steigung dParam.
   * */
double Weg::regelFktDreh(double e)
{
   double d = e*dParam
   d=d/vParam;
   return (d>(vMax/4.0)) ? vMax : ((d<(-(vMax/4.0))) ? -vMax :d);
}
  
  
  
  
}
  
  
  

