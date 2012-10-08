#include <cstdio>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <fstream>
#include <ros/ros.h>
#include <math.h>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <mobots_msgs/Pose2DPrio.h>
#include <geometry_msgs/Pose2D.h>

void* threadHelper(void*);

class WegTest {


public:
    WegTest();
    ~WegTest();

    //Subcriber
    void sollVCallback(const geometry_msgs::Pose2D &pose);

    //Publisher
    void mousePublish(const geometry_msgs::Pose2D &pose2D);
    void publishTargetPose(const mobots_msgs::Pose2DPrio &target_Pose2DPrio);
	 void pubFunction();


private:
    ros::NodeHandle nh;
    ros::Subscriber sollV_sub;

    ros::Publisher mousePose_pub;
    ros::Publisher targetPose_pub;

      int argc, mobotID;
      char argv;

      void startWegTest();
      void sollCallback(const geometry_msgs::Pose2D &soll_data);

};
