#include "hardware_driver.h"

using namespace std;


//==== method declarations ====

int main(int argc, char** argv)
{
ros::init(argc, argv, "hardware_driver");
Hardware_driver Hardware_driver(0);
}

Hardware_driver::Hardware_driver(int mobot_ID):mobotID(mobot_ID) 			//Konstruktor Weg
{
  nh = new ros::NodeHandle;
  if(!util::parseNamespace(nh->getNamespace(), mobotID))
    ROS_ERROR("%s mobotID cannot be parsed from namespace: %s", __PRETTY_FUNCTION__);
  Hardware_driver::startWeg();
}

Hardware_driver::~Hardware_driver()				//Destruktor
{
  delete nh;
}




//== begin methods ==

void Hardware_driver::startWeg()
    {
  ros::init(argc, argv, "hardware_driver");
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
      service = nh->advertiseService("setGlobalPose", &hardware_driver::ChangeGlobalPose,this);

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


    bParam=pow(vMax,rootParam)/sBrems;

    ros::param::param<double>("vParam", vParam, 1); //?? den hast du vergessen /Jonas


    initCom();
    pthread_create(&receiveThread_t, 0, receiveMethod, 0);
    ros::spin();

  }


void Hardware_driver::initCom(){
	UARTCommunication com;
	proto = new ComProtocol(&com);
	proto->protocol_init(defaultHandler);
	proto->protocol_registerHandler(SensorData_DeltaVal, sensorValHandler);
}

void* Hardware_driver::receiveMethod(void* data){
    while(1){
		proto->receiveData();
	}
	return 0;
}

void Hardware_driver::defaultHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size, Communication* com) {
	cerr << "[hardware_driver] no handler specified for id: " << id << endl;
	ROS_WARN_STREAM("[hardware_driver] no handler specified for id: " << id);
}

void Hardware_driver::sensorValHandler(enum PROTOCOL_IDS id, unsigned char *data,
		unsigned short size, Communication* com) {

	//std::cout <<"datavalHandler"<<std::endl;

	if (id != SensorData_DeltaVal) {
		std::cout << "Error, wrong ID\n" << std::endl;
		return;
	}

	int i = sizeof(struct Mouse_Data_DeltaVal);
	//printf("sizeof: %d\n", i);
	//printf("size: %d\n", size);
	if (size != i) {
		std::cout << "Error, wrong size\n" << std::endl;
		return;
	}

	struct Mouse_Data_DeltaVal *delta_vals = (struct Mouse_Data_DeltaVal*) data;
    
  geometry_msgs::Pose2D deltaPose;

     //publish
	//entweder fertig aufbereitet vom stm oder hier implementiert auf ein päärchen warten:
    
    deltaPose.x = delta_vals->;//xAvg;
    deltaPose.y = 0;//yAvg;
    deltaPose.theta = 0;//theta;
    globalPosePub.publish(deltaPose);

    std::cout << delta_vals->delta_x << std::endl;
	std::cout << delta_vals->delta_y << std::endl;
    regel();
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
  # prio == sonst: pose an der prio-position einfügen
*/
void Hardware_driver::absPoseCallback(const mobots_msgs::Pose2DPrio& next_pose){
  switch(next_pose.prio){
	 case -2: 
		targetPoses.push_back(next_pose.pose);
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
		if(prio > targetPoses.size())
		  prio = targetPoses.size();
		std::advance(it, 6);
		targetPoses.insert(it, next_pose.pose);
		break;		
  }
  
}


void Hardware_driver::relPoseCallback(const mobots_msgs::Pose2DPrio& msg){
  mobots_msgs::Pose2DPrio next;
  next.pose = globalPose;
  double cost = cos(next.pose.theta);
  double sint = sin(next.pose.theta);
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
bool Hardware_driver::changeGlobalPose(hardware_driver::ChangeGlobalPose::Request& req,
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
void Hardware_driver::regel()
{
    double eX =  sollS.x - globalPose.x;
    double eY =  sollS.y - globalPose.y;
    double eTheta =  sollS.theta - globalPose.theta;
    if (eX < minS && eY < minS && eTheta*radiusInnen < minDegree*(M_PI * radiusInneniusInnen / 180))
    { //Ziel erreicht
       Pose p={0,0,0};
       listManage(p,-2);
    }
    if (true || (wayType == STIFF) || (wayType == FAST)) //TODO
    {
     struct ServoSpeed sersp;
     sersp.s1 = regelFkt(eX); //in m und radiusInneniusInnen
     sersp.s2 = regelFkt(eY);
     sersp.s3 = regelFktDreh(eTheta);
     proto->sendData(Servo, (unsigned char*) &sersp, sizeof(struct ServoSpeed));
    }
}



/*
* Die Regelfunktion zwischen Strecke u. Geschwindigkeit stellt eine Wurzelfunktion dar.
* Der Exponent wird mittels "rootParam" festgelegt.
* bParam skaliert auf den gewünschten Bremsweg hoch, dieser kann durch bmax begrenzt werden.
* V wird metrisch verrechnet.
*/
double Hardware_driver::regelFkt(double e)
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
double Hardware_driver::regelFktDreh(double e)
{
 double d = e*dParam*radiusInneniusInnen;		//von bogenmaß auf bahngeschwindigkeit
 return (d>(vMax/4.0)) ? vMax/4.0 : ((d<(-(vMax/4.0))) ? -vMax/4.0 :d);
}


}
