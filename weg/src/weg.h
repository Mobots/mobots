#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include <mobots_msgs/Pose2DPrio.h>
#include <geometry_msgs/Pose2D.h>
#include "shutter/delta.h"
#include <math.h>
#define _USE_MATH_DEFINES
#define int vFac=0,00018025   //(Maximalgeschwindigkeit(0,18025 eines Rades/1000 in Meter! (wg. Promille)



/*
 * Der ROS-Node "Weg" übernimmt die zeitoptimale Streckenreglung und Integration der 
 * globalen Position.
 * Variablen: Bremsoptimierte Wegpunkte; Vorwärtsfahrt, Faktor für Bremsen
 *
 * Die Idee der Streckenregelung ist:
 * Rückführung über globalPose.subscribe
 * Sollgrößengenerierung über next_pose.subscribe mit Möglichkeit der Wegpunktsetzung
 * 
 * Regler: +/-Wurzel^N(+/-X) für +/-X,N=2 sollte reichen, sonst eher geringer. oder arctan
 * Begrenzt durch Betrag(Vmax).
 * 
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
  float x,y,theta;
} pose;
typedef enum{STIFF,FAST} way;

class Weg{
  
  
public:
    weg(int mobotID);
    ~weg();
    ros::Subscriber nextPose_sub;
    

  
private:
  pose globalPose, sollS, sollV;
  pose next,best;
  way wayType;
  double sBrems,bParam,vFac,rootParam, rad, vMax, minS, minDegree; //vMax in proMille < 800
 
  std::list<pose> list;
  int argc, mobotID;
  char argv;
  ros::NodeHandle nh;
  shutter::delta srv;
  
  
  
  //Subscriber
  void poseCallback(const mobots_msgs::Pose2DPrio &next_pose);
  void mouseCallback(const geometry_msgs::Pose2D &mouse_data);
  //todo Service:
 // void globalPoseCCallback(const geometry_msgs::Pose2D &cur_pose); //Korrektur-Pose nach Toro-Lauf
 
//  bool getDelta(shutter::delta::Request &req, shutter::delta::Response &res); 
// TODO potentielles Laufzeitproblem: TORO-Laufzeit länger als ein "Shutter"==> getdelta bezieht sich auf ein falsches Bild
 
  //Service
  bool changeGlobalPose(weg::changeGlobalPose::Request &req, weg::changeGlobalPose::Response &res);

  //Publisher
  void publishGlobalPose(const geometry_msgs::Pose2D &pose2D);
  void publishSollV(const geometry_msgs::Pose2D &sollV_Pose2D);
  
  
  void listManage(pose next, int prio);
  double regelFkt(double e);
  double regelFktDreh(double e);
  void startWeg();
}
