#ifndef HARDWARE_DRIVER_H
#define HARDWARE_DRIVER_H

#include "ros/ros.h"
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <sys/types.h>


#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "mobots_msgs/Pose2DPrio.h"
#include "mobots_msgs/InfraredScan.h"
#include "mobots_msgs/Twist2D.h"
#include "hardware_driver/ChangeGlobalPose.h"
#include "shutter/delta.h"

#include "../stm32vl/Client/Communication.hpp"
#include "../stm32vl/Client/ComProtocol.hpp"
#include "../stm32vl/Client/UARTCommunication.hpp"
#include "../stm32vl/mousesensor.h"

const static int POST_EVERY_X_MESSAGE=3;

/*typedef struct{
  double x,y,theta;
} Pose;*/
//typedef geometry_msgs::Pose2D Pose;
typedef enum{STIFF,FAST} way_type;

  // TODO potentielles Laufzeitproblem: TORO-Laufzeit länger als ein "Shutter"==> getdelta bezieht sich auf ein falsches Bild

    //Service später einfügen //currently unused
    //bool changeGlobalPose(hardware_driver::ChangeGlobalPose::Request &req, hardware_driver::ChangeGlobalPose::Response &res);
	
    /**
     * Receives (absolute) waypoints with a priority      */
    void absPoseCallback(const mobots_msgs::Pose2DPrio&);
    /**
     * Receives (relative i.e. delta) waypoints with a priority      */
    void relPoseCallback(const mobots_msgs::Pose2DPrio&);
    /**
     * receives servo speeds as geometry pose     */
    void speedCallback(const mobots_msgs::Twist2D&);
    /**
     * Handler method which gets called with the sensor data     */
    void sensorValHandler(enum PROTOCOL_IDS id, unsigned char *data,
            unsigned short size, Communication* com);
    /**
     * default Handler method which gets called if no other handler is set for the given id     */
    void defaultHandler(enum PROTOCOL_IDS id, unsigned char *data,
            unsigned short size, Communication* com);
    /**
     * Initialize the communication with the microcontroller     */
    void initCom();
    /**
     * Start receiving sensor data     */
    void mainLoop();
		/**
		 * stop the mobot */
		void stopMobot();


    //Subscriber
    void poseCallback(const mobots_msgs::Pose2DPrio &next_pose);
    void poseStampedCallback(const geometry_msgs::PoseStamped& next_pose);

    double regelFkt(double e);
    double regelFktDreh(double e);
    void startWeg();
    void regel();
    void correctAngle(double* theta);

    // -- values in Hz --
    geometry_msgs::Pose2D globalPose, currentTargetPose;
    std::list<geometry_msgs::Pose2D> targetPoses;


    ros::Subscriber nextPoseSubRel, nextPoseSubAbs, speedSub;
    ros::Publisher mousePosePub, globalPosePub, infraredScanPub;
    ros::ServiceClient shutterClient;
    ros::ServiceServer setGlobalPoseServer;
    ComProtocol *proto;
		UARTCommunication com;
    bool received;
    pthread_t receiveThread_t;

    ros::NodeHandle* nh;

    ros::Publisher pose2D_pub;
    ros::Publisher sollV_pub;

		//currently unused
    /*ros::ServiceClient client;
    ros::ServiceServer service;
    shutter::delta srv;*/
		
		//the mouse data which gets sent to the shutter
		geometry_msgs::Pose2D mouse;
		
		way_type wayType;
		double sBrems,bParam,vFac,dParam,rootParam, radiusInnen, vMax, minS, minDegree,drehFac;
		//the integrated mouse values before they get sent
		double dX, dY, dTheta;

		int mobotID, counter;
		int mainLoopFrequency;
		
		bool velocityControlled;


#endif // HARDWARE_DRIVER_H
