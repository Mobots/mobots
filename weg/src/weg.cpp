#include "weg.h"


Weg::Weg(int mobotID):this.mobotID(mobotID) 			//Konstruktor Weg
{
  argc = 0;
  std::stringstream s;
  s << "weg_" << mobot_ID;
  ros::init(argc, (char**)argv, s.str());
  ros::NodeHandle nh;
  Weg::startWeg();
}

Weg::~Weg()				//Destruktor
{
}

void Weg::startWeg()
{
  ROS_INFO("Mobot %d: Weg angeben",mobotID);
  nextPose_sub = nh.subscribe("mobot_pose/waypoint", 30, &Weg::poseCallback, this);
  pose2D_pub = nh.advertise<geometry_msgs::Pose2D>("mobot_pose/globalPose", 5);
  sollV_pub = nh.advertise<geometry_msgs::Pose2D>("driver/sollV", 2);

  //Services
  //ros::ServiceClient = nh.serviceClient<shutter::delta>("mobot_pose/getDelta");
  ros::ServiceCenter service = nh.advertiseService("mobot_pose/changeGlobalPose", changeGlobalPose);

  //Parameter übernehmen
  ros::param::param<double>("sBrems",sBrems,0.2);
  ros::param::param<double>("rootParam",rootParam,2.0);
  ros::param::param<double>("rootParam",bParam,0,06);   //calculate: bParam=vMax/4/(desired start-to-break-point in degrees)*180/pi/radius
  // ros::param::param<double>("vFac",vFac,1);  //anderes Konzept
  //vFac ist der zusammenhang: Vmaximal/1000 zwischen promilledaten und realität
  ros::param::param<double>("rad",rad,0.14); //TODO, genauer Radius, messen Mitte- Räder-Bodenkontakt
  ros::param::param<double>("vMax",vMax,0.15); //theoretisch maximal 0.18, weniger, um nicht mit maximum zu laufen
  ros::param::param<double>("minS",minS,0.02);	//Toleranz für erreichten Wegpunkt
  ros::param::param<double>("minDegree",minDegree,1); //Toleranz für erreichte Drehrichtung


  bParam=pow(vMax,rootParam)/sBrems;

  ros::spin();
}

void Weg::poseCallback(const mobots_msgs::Pose2DPrio &next_pose)
{
  listManage({ next_pose.x, next_pose.y, next_pose.theta}, next_pose.prio);
}


 /*listmanage organisiert die nächsten wegpunkte, verschiedenen angabemöglichkeiten:
  * prio== -2: der erste eintrag wird gelöscht (zB erledigt)
  * prio== -1: gesamte liste löschen, pose alleine einfügen
  * prio== 0 : pose vor alle anderen einfügen
  * prio== 1 : pose am ende der liste einfügen
  * prio== sonst: pose an der prio-position einfügen
  */
void Weg::listManage(pose next, int prio)
{
  bool flag = false;
  switch(prio)
  {
    case -2:
      list::pop_front();
      flag = true;
      break;
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
      if (list.size() < 2) {     flag = true; }
    break;
    default:
    if (list.size() > prio) {
      list::insert(prio, next);
    } else {
      list::push_back(next);
      if (list.size() < 2) {     flag = true; }
    }
    break;
  }

  if (flag)  {
    //Fall: Nr0 oder Nr1 aktualisiert, Nr1 wegen Bremsvermeidung.
    sollS = list::front();
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
/*************************************************************************************
 *
 *
 **************************************************************************************/
void Weg::regel()
{
 double eX =  sollS.x - globalPose.x;
 double eY =  sollS.y - globalPose.y;
 double eTheta =  sollS.theta - globalPose.theta;
 if (eX < minS && eY < minS && eTheta*rad < minDegree*(M_Pi * rad / 180)) { //Ziel erreicht
   listManage(NULL,-2);
 }
 if (wayType == STIFF) || (wayType == FAST) //TODO
 {
   geometry_msgs::Pose2D sollV;
   sollV.x=regelFkt(sollS.x);
   sollV.y=regelFkt(sollS.y);
   sollV.theta=regelFktDreh(sollS.theta); //Weitergabe Regeldifferenz im Bogenmaß
   sollV_pub.publish(sollV);
 }
}



/*
 * Die Regelfunktion zwischen Strecke u. Geschwindigkeit stellt eine Wurzelfunktion dar.
 * Der Exponent wird mittels "rootParam" festgelegt.
 * bParam skaliert auf den gewünschten Bremsweg hoch, dieser kann durch bmax begrenzt werden.
 * V wird metrisch verrechnet.
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
   d=d/vParam;
   return (d<-vMax) ? -vMax :d;
 }

}

  /*
   * Linear steigende Regelfunktion, Steigung dParam.
   * */
double Weg::regelFktDreh(double e)
{
   double d = e*dParam*rad;		//von bogenmaß auf bahngeschwindigkeit
   return (d>(vMax/4.0)) ? vMax/4.0 : ((d<(-(vMax/4.0))) ? -vMax/4.0 :d);
}




