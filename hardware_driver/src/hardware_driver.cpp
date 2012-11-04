#include "hardware_driver.h"
#include "mobots_common/utils.h"

using namespace std;

int main(int argc, char** argv)
{
ros::init(argc, argv, "hardware_driver");
  nh = new ros::NodeHandle;
  if(!mobots_common::utils::parseNamespace(nh->getNamespace(), mobotID))
    ROS_ERROR("%s mobotID cannot be parsed from namespace: %s", __PRETTY_FUNCTION__, nh->getNamespace().c_str());
  startWeg();
}

//== begin methods ==

void startWeg()
{
  nh = new ros::NodeHandle;

  //shutterClient = nh->serviceClient<shutter::delta>("getDelta");
  setGlobalPoseServer = nh->advertiseService("set_pose", changeGlobalPose);

    ROS_INFO("Mobot %d: Weg angeben", mobotID);

    //Publisher and Subscriber
    nextPoseSubRel = nh->subscribe("waypoint_rel", 5, relPoseCallback);
    nextPoseSubAbs = nh->subscribe("waypoint_abs", 5, absPoseCallback);
    //speedSub = nh->subscribe("velocity", 2, sendSpeedCallback); interessiert eigntl nicht

    mousePosePub = nh->advertise<geometry_msgs::Pose2D>("mouse", 1);
    //globalPosePub = nh->advertise<geometry_msgs::Pose2D>("pose", 2);
    infraredScanPub = nh->advertise<mobots_msgs::InfraredScan>("infrared", 2); //TODO Handler dafür


    //Services
      client = nh->serviceClient<shutter::delta>("getDelta");
      service = nh->advertiseService("setGlobalPose", changeGlobalPose);

    //Parameter übernehmen
    ros::param::param<double>("sBrems",sBrems,0.2);
    ros::param::param<double>("rootParam",rootParam,2.0);
    //ros::param::param<double>("bParam",bParam,0.06);   //calculate: bParam=vMax/4/(desired start-to-break-point in degrees)*180/pi/radiusInneniusInnenius
    ros::param::param<double>("radiusInnen",radiusInnen,0.14); //TODO, genauer radiusInnenius, messen Mitte- Räder-Bodenkontakt
    ros::param::param<double>("vMax",vMax,0.15); //theoretisch maximal 0.18, weniger, um nicht mit maximum zu laufen
    ros::param::param<double>("minS",minS,0.02);	//Toleranz für erreichten Wegpunkt
    ros::param::param<double>("minDegree",minDegree,1); //Toleranz für erreichte Drehrichtung
    // ros::param::param<double>("vFac",vFac,1);  //anderes Konzept
    //vFac ist der zusammenhang: Vmaximal/1000 zwischen promilledaten und realität
    ros::param::param<double>("vParam", vParam, 1); //?? den hast du vergessen /Jonas
    ros::param::param<way_type>("fahrTyp",wayType,FAST);

    bParam=pow(vMax,rootParam)/sBrems;

    ge

    initCom();
    pthread_create(&receiveThread_t, 0, receiveMethod, 0);
    counter=0; //used to send mouse deltas every XXX incoming message
    ros::spin();

  }


void initCom(){
	proto = new ComProtocol(&com);
	proto->protocol_init(defaultHandler);
	proto->protocol_registerHandler(SensorData_transformedDelta, sensorValHandler);
}

void* receiveMethod(void* data){
    while(1){
		proto->receiveData();
	}
	return 0;
}

void defaultHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size, Communication* com) {
	cerr << "[hardware_driver] no handler specified for id: " << id << endl;
	ROS_WARN_STREAM("[hardware_driver] no handler specified for id: " << id);
}

void sensorValHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size, Communication* com) {


	//std::cout <<"datavalHandler"<<std::endl;

	if (id == SensorData_transformedDelta) {
		counter++;
			//check:
		int i = sizeof(struct Mouse_Data_Delta2DPose);
		if (size != i) {
			std::cout << "Error, wrong size\n" << std::endl;
			return;
		}
		struct Mouse_Data_Delta2DPose *delta_vals = (struct Mouse_Data_Delta2DPose*) data;
		 //publish
		geometry_msgs::Pose2D deltaPose;

		deltaPose.x += delta_vals->delta_x;//xAvg;
		deltaPose.y += delta_vals->delta_y;//yAvg;
		deltaPose.theta += delta_vals->delta_theta;//theta;*/
					//TODO globalPose aktualisieren
	    globalPose.x+=deltaPose.x;
	    globalPose.y+=deltaPose.y;
	    globalPose.theta+=deltaPose.theta;
		deltaPose.x = deltaPose.y = deltaPose.y = 0;
		if (POST_EVERY_X_MESSAGE == counter) {
			counter=0;

			globalPosePub.publish(globalPose);
		}

		if (!targetPoses.empty()) {
			regel();
		}

	} else {
		//case: no mouse delta vals
				return;
		}
}

/*
void* infraredReader(void* data){
  ros::Rate rate(infraredFrequency);
  ros::Publisher pub = nh->advertise<>("", 2);
  while(ros::ok()){
    read(infraredFD, XX, X);
    pub.publish(XX);
    rate.sleep();
  }
}*/




/**
 *# prio == 0: pose vor allen anderen einfügen, rest verwerfen (default)
  # prio == -1: pose vor allen anderen einfügen
  # prio == -2: pose am ende der liste einfügen
  # prio == sonst: pose an der prio-position einfügen (zero based)
*/
void absPoseCallback(const mobots_msgs::Pose2DPrio& next_pose){
  switch(next_pose.prio){
	 case -2: 
		targetPoses.push_back(next_pose.pose);
		if (targetPoses.size() == 1) { //case: first element was just inserted
			currentTargetPose = next_pose.pose;
		}
		break;
	 case -1:
		targetPoses.push_front(next_pose.pose);
		currentTargetPose = next_pose.pose;
		break;
	 case 0:
		targetPoses.clear();
		targetPoses.push_back(next_pose.pose);
		currentTargetPose = next_pose.pose;
		break;
	 default:
		list<geometry_msgs::Pose2D>::iterator it = targetPoses.begin();
		int prio = next_pose.prio;
		if(prio >= targetPoses.size()) {
			targetPoses.push_back(next_pose.pose);
			break;
		}
		std::advance(it, prio);	//keep this zero-based, user gives prio==5, it goes to list[5]=> inserted as6th element
		targetPoses.insert(it, next_pose.pose);
		break;		
  }
  
}


void relPoseCallback(const mobots_msgs::Pose2DPrio& msg){

  //retrieve position, to sum up with new delta pose:
  mobots_msgs::Pose2DPrio next;
  if (msg.prio == -1 || msg.prio == 0 || targetPoses.empty()) 	//current position, if set as current target positon or list empty
  {
	  next.pose = globalPose;
  } else if (msg.prio == -2) {	//last position
	  next.pose= targetPoses.back();
  } else { // prio >=1 (default) s.o.
	  next.pose = globalPose;
	  int prio = next_pose.prio;
	  if(prio >= targetPoses.size())
			prio = targetPoses.size()-1;	//retrieve last element
	  std::advance(it, prio);
		  next.pose = *it;
  }
  double cost = cos(globalPose.theta);
  double sint = sin(globalPose.theta);
  next.pose.x += cost*msg.pose.x - sint*msg.pose.y;
  next.pose.y += sint*msg.pose.x + cost*msg.pose.y;
  next.pose.theta += msg.pose.theta;
  next.prio = msg.prio;
  absPoseCallback(next);
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
bool changeGlobalPose(hardware_driver::ChangeGlobalPose::Request& req,
							 hardware_driver::ChangeGlobalPose::Response& res){
  /*shutter::delta srv;
  if (1client.call(srv))
  {						//LOCK ???
   globalPose.x=srv.response.x+req.x;
   globalPose.y=srv.response.y+req.y;
   globalPose.theta=srv.response.theta+req.theta;*/
  if(false){
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




//Streckenregelung: Strecken in m
/*************************************************************************************
*
*
**************************************************************************************/
void regel()
{
    double eX =  currentTargetPose.x - globalPose.x;
    double eY =  currentTargetPose.y - globalPose.y;
    double eTheta=0;
    if (wayType == FAST) {
    	eTheta =  tan(eY/eX) - M_PI/2 - globalPose.theta; 		//-Pi/2, um ziel mit y als hauptfahrrichtung anzufahren
    } else {
        eTheta =  currentTargetPose.theta - globalPose.theta;
    }
    if (eX < minS && eY < minS && eTheta*radiusInnen < minDegree*(M_PI * radiusInnen / 180))
    { //Ziel erreicht
       //geometry_msgs::Pose2D p={0,0,0};
			targetPoses.pop_front(); //aktuelles ziel erreicht => aus liste löschen
			if(!targetPoses.empty())
				currentTargetPose = targetPoses.front();
			else
				currentTargetPose = globalPose; //bleiben wir halt stehn
		     struct ServoSpeed sersp;
		     sersp.s1 = 0;
		     sersp.s2 = 0;
		     sersp.s3 = 0;
		     proto->sendData(Debug_Controller, (unsigned char*) &sersp, sizeof(struct ServoSpeed));
			return;
    }
     struct ServoSpeed sersp;
     sersp.s1 = regelFkt(eX); //in meter und rad
     sersp.s2 = regelFkt(eY);
     sersp.s3 = regelFktDreh(eTheta);
     proto->sendData(Debug_Controller, (unsigned char*) &sersp, sizeof(struct ServoSpeed));

}



/*
* Die Regelfunktion zwischen Strecke u. Geschwindigkeit stellt eine Wurzelfunktion dar.
* Der Exponent wird mittels "rootParam" festgelegt.
* bParam skaliert auf den gewünschten Bremsweg hoch, dieser kann durch bmax begrenzt werden.
* V wird metrisch verrechnet.
*/
double regelFkt(double e)
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
double regelFktDreh(double e)
{
 double d = e*dParam*radiusInnen;		//von bogenmaß auf bahngeschwindigkeit //TODO so war das vorher  e*dParam*=radiusInnen;
 return (d>(vMax/4.0)) ? vMax/4.0 : ((d<(-(vMax/4.0))) ? -vMax/4.0 :d);
}
