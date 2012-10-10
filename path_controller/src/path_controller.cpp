#include "path_controller.h"
#include "../../util/util.h"

int main(int argc, char** argv)
{
ros::init(argc, argv, "PathController");
PathController weg(0);
}

PathController::PathController(int mobot_ID):mobotID(mobot_ID) 			//Konstruktor Weg
{
  nh = new ros::NodeHandle;
  if(!util::parseNamespace(nh->getNamespace(), mobotID))
    ROS_ERROR("%s mobotID cannot be parsed from namespace: %s", "path_controller", nh->getNamespace().c_str());
  PathController::startWeg();
}

PathController::~PathController()				//Destruktor
{
  delete nh;
}

void PathController::startWeg()
{
  ROS_INFO("Mobot %d: Weg angeben", mobotID);
  nextPose_sub = nh->subscribe("waypoint_prioritized", 30, &PathController::poseCallback, this);
  nextStampedPose_sub = nh->subscribe("waypoint", 2, &PathController::poseStampedCallback, this);
  mousePose_sub = nh->subscribe("mouse", 100, &PathController::mouseCallback, this);
  pose2D_pub = nh->advertise<geometry_msgs::Pose2D>("globalPose", 5);
  sollV_pub = nh->advertise<geometry_msgs::Pose2D>("velocity", 2);


  //Services
    client = nh->serviceClient<shutter::delta>("getDelta");
    service = nh->advertiseService("setGlobalPose", &PathController::changeGlobalPose,this);

  //Parameter übernehmen
  ros::param::param<double>("sBrems",sBrems,0.2);
  ros::param::param<double>("rootParam",rootParam,2.0);
  //ros::param::param<double>("bParam",bParam,0.06);   //calculate: bParam=vMax/4/(desired start-to-break-point in degrees)*180/pi/radius
  ros::param::param<double>("rad",rad,0.14); //TODO, genauer Radius, messen Mitte- Räder-Bodenkontakt
  ros::param::param<double>("vMax",vMax,0.15); //theoretisch maximal 0.18, weniger, um nicht mit maximum zu laufen
  ros::param::param<double>("minS",minS,0.02);	//Toleranz für erreichten Wegpunkt
  ros::param::param<double>("minDegree",minDegree,1); //Toleranz für erreichte Drehrichtung
  // ros::param::param<double>("vFac",vFac,1);  //anderes Konzept
  //vFac ist der zusammenhang: Vmaximal/1000 zwischen promilledaten und realität


  bParam=pow(vMax,rootParam)/sBrems;
  
  ros::param::param<double>("vParam", vParam, 1); //?? den hast du vergessen /Jonas

  ros::spin();
}

void PathController::poseCallback(const mobots_msgs::Pose2DPrio &next_pose)
{
    double x=next_pose.pose.x;
    double y=next_pose.pose.y;
    double theta=next_pose.pose.theta;
    Pose p={x,y,theta};
    listManage(p, next_pose.prio);
}

void PathController::poseStampedCallback(const geometry_msgs::PoseStamped& next_pose){
    double x = next_pose.pose.position.x;
    double y = next_pose.pose.position.y;
    double theta = 2*acos(next_pose.pose.orientation.w);
    Pose p = {x,y,theta};
    listManage(p, -1);
}


 /*listmanage organisiert die nächsten wegpunkte, verschiedenen angabemöglichkeiten:
  * prio== -2: der erste eintrag wird gelöscht (zB erledigt)
  * prio== -1: gesamte liste löschen, pose alleine einfügen
  * prio== 0 : pose vor alle anderen einfügen
  * prio>= 1 : pose am ende der liste einfügen
  * prio== sonst: pose an der prio-position einfügen
  */
void PathController::listManage(Pose next, int prio)
{
  bool flag = false;
  switch(prio)
  {
    case -2:
      list.pop_front();
      flag = true;
      break;
    case -1:
      list.clear();
      list.push_front(globalPose);
      flag = true;
    break;
    case 0:
      list.push_front(next);
      flag = true;
    break;
    default:
      list.push_back(next);
      if (list.size() < 2) {     flag = true; }
    break;
  }

  if (flag)  {
    //Fall: Nr0 oder Nr1 aktualisiert, Nr1 wegen Bremsvermeidung.
    sollS = list.front();
  }
}

/***********************************************************************************
 * changeGlobalPose kann zwecks aktualisierung der jeweiligen globalen mobot-pose
 * aufgerufen werden. zB durch toro, wenn dieser eine frische fehlerfreiere Position
 * als die des Maussensor-Integrals errechnet hat. Die Methode besorgt die delta-beträge
 * vom shutter, da sich die aktualisierte globalpose auf den letzten "shut" bezieht.
 *
 * Dieser Ablauf ist eventuell systematisch fehlerbehaftet, falls in der Toro-Rechenzeit ein
 * erneuter "shut" auftritt. Dieser Fall sollte dann vorm globalPose-AUfruf durch den Toro-
 * Node überprüft werden, der dann auch das Delta vom shutter besorgen  und bereits
 * aufadiert an diesen Node weitergeben sollte
 *****************************************************************************************/
bool PathController::changeGlobalPose(path_controller::ChangeGlobalPose::Request &req,
                           path_controller::ChangeGlobalPose::Response &res)
{
  shutter::delta srv;
  if (client.call(srv))
  {						//LOCK ???
   globalPose.x=srv.response.x+req.x;
   globalPose.y=srv.response.y+req.y;
   globalPose.theta=srv.response.theta+req.theta;
  } else {
    ROS_ERROR("Failed to call getDelta");
    //change to request-values, as this is ours best bet
    globalPose.x=req.x;
    globalPose.y=req.y;
    globalPose.theta=req.theta;
    return false;   //t
  }
  return true;

}


/***********************************************************************************
 * mouseCallback addiert die Delta-Posen des Hardware-Drivers auf.
 * Diese wird als Globalpose gepublisht.
 * Für jeden Callback-Aufruf (Driver sollte 100Hz durchziehen) wird geregelt
 ***********************************************************************************/
void PathController::mouseCallback(const geometry_msgs::Pose2D &mouse_data) {
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
/*************************************************************************************
 *
 *
 **************************************************************************************/
void PathController::regel()
{
 double eX =  sollS.x - globalPose.x;
 double eY =  sollS.y - globalPose.y;
 double eTheta =  sollS.theta - globalPose.theta;
 if (eX < minS && eY < minS && eTheta*rad < minDegree*(M_PI * rad / 180)) { //Ziel erreicht
     Pose p={0,0,0};
     listManage(p,-2);
 }
 if (true || (wayType == STIFF) || (wayType == FAST)) //TODO
 {
   geometry_msgs::Pose2D sollV;
   sollV.x=regelFkt(eX); //hier war wohl n typo /Jonas
   sollV.y=regelFkt(eY);
   sollV.theta=regelFktDreh(eTheta); //Weitergabe Regeldifferenz im Bogenmaß
   sollV_pub.publish(sollV);
 }
}



/*
 * Die Regelfunktion zwischen Strecke u. Geschwindigkeit stellt eine Wurzelfunktion dar.
 * Der Exponent wird mittels "rootParam" festgelegt.
 * bParam skaliert auf den gewünschten Bremsweg hoch, dieser kann durch bmax begrenzt werden.
 * V wird metrisch verrechnet.
 */
double PathController::regelFkt(double e)
{
 if (e>0) {
   double d = pow(e*bParam, 1.0/rootParam);
   d=d/vParam;
   return (d>vMax) ? vMax :d;
 }
 if (e<0) {
   double d =-(pow(-e*bParam, 1.0/rootParam));
   d=d/vParam;
   return (d<-vMax) ? -vMax :d;
 }
    return 0;
}

  /*
   * Linear steigende Regelfunktion, Steigung dParam.
   * */
double PathController::regelFktDreh(double e)
{
   double d = e*dParam*rad;		//von bogenmaß auf bahngeschwindigkeit
   return (d>(vMax/4.0)) ? vMax/4.0 : ((d<(-(vMax/4.0))) ? -vMax/4.0 :d);
}
