#include "hardware_driver.h"
#include "mobots_common/utils.h"
#include "signal.h"

using namespace std;

void sigHandler(int signum){
	stopMobot();
  exit(0);
}

int main(int argc, char** argv)
{
ros::init(argc, argv, "hardware_driver");
  nh = new ros::NodeHandle;
  if(!mobots_common::utils::parseNamespace(nh->getNamespace(), mobotID))
    ROS_ERROR("%s mobotID cannot be parsed from namespace: %s", __PRETTY_FUNCTION__, nh->getNamespace().c_str());
	signal(SIGINT, sigHandler);
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
    speedSub = nh->subscribe("velocity", 5, speedCallback);

    mousePosePub = nh->advertise<geometry_msgs::Pose2D>("mouse", 5);
    globalPosePub = nh->advertise<geometry_msgs::Pose2D>("pose", 2);
    //infraredScanPub = nh->advertise<mobots_msgs::InfraredScan>("infrared", 5); //TODO Handler dafür


    //Services werden aktuell nicht benutzt
      //client = nh->serviceClient<shutter::delta>("getDelta");
      //service = nh->advertiseService("setGlobalPose", changeGlobalPose);

    //Parameter übernehmen
    ros::param::param<double>("sBrems",sBrems,0.2);
    ros::param::param<double>("rootParam",rootParam,2.0);
    ros::param::param<double>("dParam",dParam,0.4297);   //calculate: bParam=vMax/4/(desired start-to-break-point in degrees)*180/pi/radiusInnen
    ros::param::param<double>("radiusInnen",radiusInnen,0.102); //TODO, genauer radiusInnenius, messen Mitte- Räder-Bodenkontakt
    ros::param::param<double>("vMax",vMax,1); //theoretisch maximal 0.18, weniger, um nicht mit maximum zu laufen
    ros::param::param<double>("minS",minS,0.02);	//Toleranz für erreichten Wegpunkt
    ros::param::param<double>("minDegree",minDegree,1); //Toleranz für erreichte Drehrichtung
    ros::param::param<double>("vFac", vFac, 0.00015);     //vFac ist der zusammenhang: Vmaximal/1000 zwischen promilledaten und realität
		ros::param::param<int>("mainLoopFrequency", mainLoopFrequency, 20);     //vFac ist der zusammenhang: Vmaximal/1000 zwischen promilledaten und realität

    string wayTypeString;
    ros::param::param<string>("fahrTyp",wayTypeString, "FAST");
		
	for(int i = 0; i < wayTypeString.size(); i++) {
		wayTypeString[i] = toupper(wayTypeString[i]);
	}
	if(wayTypeString == string("FAST")) {
		ros::param::param<double>("drehFac",drehFac,0.4);	//Bruchteil der vMax, mit der sich der mobot maximal drehen soll
		wayType = FAST;
	}
	else if(wayTypeString == string("STIFF")) {
		ros::param::param<double>("drehFac",drehFac,0.25);	//Bruchteil der vMax, mit der sich der mobot maximal drehen soll
		wayType = STIFF;
	}
	else
		ROS_ERROR("unknown type %s", wayTypeString.c_str());


    bParam=pow(vMax,rootParam)/sBrems;

    initCom();
		mainLoop();
    counter=0; //used to send mouse deltas every XXX incoming message
    ros::spin();

  }


void initCom(){
	proto = new ComProtocol(&com);
	proto->protocol_init(defaultHandler);
	proto->protocol_registerHandler(MOUSE_DATA, sensorValHandler);
}

//gets it from the microcontroller
void mainLoop(){
  ros::Rate rate(mainLoopFrequency); 
	while(1){
		ros::spinOnce();
		proto->receiveData();
		rate.sleep();
	}
}

void defaultHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size, Communication* com) {
	cerr << "[hardware_driver] no handler specified for id: " << id << endl;
	ROS_WARN_STREAM("[hardware_driver] no handler specified for id: " << id);
}

void sensorValHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size, Communication* com) {


	//std::cout <<"datavalHandler"<<std::endl;

	if (id == MOUSE_DATA) {
		counter++;
			//check:
		int i = sizeof(struct MouseData);
		if (size != i) {
			std::cout << "Error, wrong size\n" << std::endl;
			return;
		}
		struct MouseData *delta_vals = (struct MouseData*) data;
		 //publish
		
		//cout << delta_vals->x << ' ' << delta_vals->y << ' ' << delta_vals->theta << endl;



//***************************+++ Transform +++********************************
    //mobot-> global
  double cost = cos(-globalPose.theta);
  double sint = sin(-globalPose.theta);
	double x = delta_vals->x;
	double y = delta_vals->y;
	delta_vals->x = cost*x - sint*y;
  delta_vals->y = sint*x + cost*y;



//****************************************************************************
			globalPose.x += delta_vals->x;
			globalPose.y += delta_vals->y;
			globalPose.theta += delta_vals->theta;
			correctAngle(*(&globalPose.theta));

        if (POST_EVERY_X_MESSAGE == counter) { //yoda condition 
				counter=0;
				geometry_msgs::Pose2D mouse;
				mouse.x = delta_vals->x;
				mouse.y = delta_vals->y;
				mouse.theta = delta_vals->theta;
				mousePosePub.publish(mouse);
				globalPosePub.publish(globalPose);
			}			
		

		if (!targetPoses.empty()) {
			if(!velocityControlled)
				regel();
		}

	} else {
		//case: no mouse delta vals
				return;
		}
}


void correctAngle(double& theta) {
	if(theta < -M_PI)
		theta += 2*M_PI;
	else if(theta > M_PI)
		theta -= 2*M_PI;
}

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
  cout << "absPoseCallback new target: x " << currentTargetPose.x << " y " << currentTargetPose.y << " theta " << currentTargetPose.theta << endl;
	cout << "current globalPose: x " << globalPose.x << " y " << globalPose.y << " theta " << globalPose.theta << endl;
  regel();
}


void relPoseCallback(const mobots_msgs::Pose2DPrio& msg){
	cout << "relPoseCallback x " << msg.pose.x << " y " << msg.pose.y << " theta " << msg.pose.theta << endl;
  //retrieve position, to sum up with new delta pose:
  mobots_msgs::Pose2DPrio next;
  if (msg.prio == -1 || msg.prio == 0 || targetPoses.empty()) 	//current position, if set as current target positon or list empty
  {
	  next.pose = globalPose;
  } else if (msg.prio == -2) {	//last position
	  next.pose= targetPoses.back();
  } else { // prio >=1 (default) s.o.
	  int prio = msg.prio-1;
		list<geometry_msgs::Pose2D>::iterator it = targetPoses.begin();
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
    double dist= sqrt(eX*eX+eY*eY);
    double eTheta=0;
    if (wayType == FAST) {
        if ( eX != 0) {
        	eTheta =  tan(eY/eX) - M_PI/2 - globalPose.theta; 		//-Pi/2, um ziel mit y als hauptfahrrichtung anzufahren
        } else eTheta = 0 - M_PI/2 - globalPose.theta;
    } else {
        eTheta =  currentTargetPose.theta - globalPose.theta;
    }
    correctAngle(eTheta);    // correct with 2Pi problem: (this also guarentees to turn optimal)
    if (dist < minS && eTheta*radiusInnen < minDegree*(M_PI * radiusInnen / 180)){ //Ziel erreicht
			cout << "reached waypoint: x " << currentTargetPose.x << " y " << currentTargetPose.y << " theta " << currentTargetPose.theta << endl;
       //geometry_msgs::Pose2D p={0,0,0};
			targetPoses.pop_front(); //aktuelles ziel erreicht => aus liste löschen
			if(!targetPoses.empty())
				currentTargetPose = targetPoses.front();
			else {
				currentTargetPose = globalPose; //bleiben wir halt stehn
				stopMobot();
			}
			return;
    }
     struct Velocity vel;
     double vel_ges =regelFkt(dist);
    if (dist != 0) {
        vel.x = eX*vel_ges/dist;//Strahlensatz //in meter/s, s3 ist auch bahngeschwindigkeit, dann /vFac zur skalierung für servo geschw.
        vel.y = eY*vel_ges/dist;
    } 
//***************************+++ Transform +++********************************
    
  double cost = cos(globalPose.theta);
  double sint = sin(globalPose.theta);
	double x = vel.x;
	double y = vel.y;
  vel.y = sint*x + cost*y;
  vel.x = cost*x - sint*y;
	
	cout << "eX " << eX << " eY " << eY << " eTheta " << eTheta << " || ";
	cout << "velx " << vel.x << " vely " << vel.y << " veltheta " << vel.theta << endl;



//****************************************************************************
     vel.theta = regelFktDreh(eTheta);
     proto->sendData(VELOCITY, (unsigned char*) &vel, sizeof(struct Velocity));
}

void stopMobot(){
	struct Velocity vel;
	vel.x = 0;
	vel.y = 0;
	vel.theta = 0;
	proto->sendData(VELOCITY, (unsigned char*) &vel, sizeof(struct Velocity));
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
 return (d>vMax) ? vMax :d;
}
if (e<0) {
 double d =-(pow(-e*bParam, 1.0/rootParam));
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
 return (d>(vMax*drehFac)) ? (vMax*drehFac) : ((d<(-(vMax*drehFac))) ? (-(vMax*drehFac)) :d);
}

void speedCallback(const mobots_msgs::Twist2D& msg){
	if(msg.x > 1){ //values are normalized between [-1,1]
		velocityControlled = false;
		cerr << "disabling velocity control" << endl;
		regel();
		return;
	}
	velocityControlled = true;
	struct Velocity vel;
	vel.x = msg.x;
	vel.y = msg.y;
	vel.theta = msg.theta;
	//cerr << "vel x " << msg.x << " y " << msg.y << " theta " << msg.theta << endl;
	proto->sendData(VELOCITY, (unsigned char*) &vel, sizeof(struct Velocity));
}
