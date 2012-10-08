#include <cstdio>
#include <iostream>
#include <list>
#include <ros/ros.h>
#include <mobots_msgs/Pose2DPrio.h>
#include <geometry_msgs/Pose2D.h>
#include "shutter/delta.h"
#include "weg/ChangeGlobalPose.h"
#include <math.h>
#define _USE_MATH_DEFINES
const double vFac = 0.00018025;   //(Maximalgeschwindigkeit(0,18025 eines Rades/1000 in Meter! (wg. Promille)



/*
 * Der ROS-Node "Weg" übernimmt die zeitoptimale Streckenreglung und Integration der
 * globalen Position.
 *
 * Die Idee der Streckenregelung ist:
 * Rückführung über globalPose.subscribe
 * Sollgrößengenerierung über next_pose.subscribe mit Möglichkeit der Wegpunktsetzung
 *
 * Regler: +/-Wurzel^N(+/-X) für +/-X,N=2 sollte reichen, sonst eher geringer. oder arctan
 * Begrenzt durch Betrag(Vmax).
 *
 * wird eher nicht mehr umgesetzt:
 * Bremsoptimierte Wegpunkte: dazu Sollgrößengenerierung/Reglermanipulation zum günstigen
 * Wegpunkt-Abfahren evntl Auswertung der Folgerichtung: (Vmin setzen)
 * -Fahrtrichtung +/-15° nur 20%Reduktion
 * -Fahrtrichtung +/- 60° nur 50%Reduktion
 * -Fahrtrichtung sonst: volles Abbremsen
 *
 * Drehgeschwindigkeit auf 20%/30%-Vmax begrenzen. Bekommt im Geschw.-Regler Vorrang!
 *
 *
 * */

typedef struct{
  double x,y,theta;
} Pose;
typedef enum{STIFF,FAST} way;



class Weg {

public:
    Weg(int mobot_ID);
    ~Weg();


    //Subscriber
    void poseCallback(const mobots_msgs::Pose2DPrio &next_pose);
    void mouseCallback(const geometry_msgs::Pose2D &mouse_data);

  // TODO potentielles Laufzeitproblem: TORO-Laufzeit länger als ein "Shutter"==> getdelta bezieht sich auf ein falsches Bild

    //Service
    bool changeGlobalPose(weg::ChangeGlobalPose::Request &req, weg::ChangeGlobalPose::Response &res);

    //Publisher
    void publishGlobalPose(const geometry_msgs::Pose2D &pose2D);
    void publishSollV(const geometry_msgs::Pose2D &sollV_Pose2D);

private:
  /* Auf dem Stack allokieren geht nicht, weil dann wird es vor ros::init erzeugt,
	* Achtung: wird das letzte NodeHandle deallokiert, wird der Node heruntergefahren, 
	* die Benutzung von publisher/subscriber führt dann zu nicht nachvollziehbaren SegFaults. -_- */
	 ros::NodeHandle* nh;
	 
    ros::Subscriber nextPose_sub;
    ros::Subscriber mousePose_sub;

    ros::Publisher pose2D_pub;
    ros::Publisher sollV_pub;

    ros::ServiceClient client;
    ros::ServiceServer service;
    shutter::delta srv;


      Pose globalPose, sollS;
      Pose next,best;
      std::list<Pose> list;
      way wayType;
      double sBrems,bParam,vParam,dParam,rootParam, rad, vMax, minS, minDegree;

      int argc, mobotID;
      char argv;

      double regelFkt(double e);
      double regelFktDreh(double e);
      void listManage(Pose next, int prio);
      void startWeg();
      void regel();
};
