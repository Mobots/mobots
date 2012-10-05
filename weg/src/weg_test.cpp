#include "weg_test.h"


int main(int argc, char** argv)
{
ros::init(argc, argv, "weg");
}

WegTest::WegTest() 			//Konstruktor Weg
{
  argc = 0;
  std::stringstream s;
  ros::init(argc, (char**)argv, s.str());
  ros::NodeHandle nh;
  startWegTest();
}

WegTest::~WegTest()				//Destruktor
{
}

void WegTest::startWegTest()
{
    //subscribes to
    sollV_sub = nh.subscribe("driver/sollV", 30, &WegTest::sollCallback, this);

    mousePose_pub = nh.advertise<geometry_msgs::Pose2D>("mouse/pose", 5);
    targetPose_pub = nh.advertise<mobots_msgs::Pose2DPrio>("mobot_pose/waypoint", 2);

    //set target to get to
    mobots_msgs::Pose2DPrio pub_pose;
    pub_pose.pose.x = 10;
    pub_pose.pose.y = 10;
    pub_pose.pose.theta = 0;
    pub_pose.prio = 0;
    targetPose_pub.publish(pub_pose);

    //start process to publish mousedata with 100hz

    boost::thread pubThread(pubFunction);
    pubThread.join();

    ros::spin();
}

void WegTest::pubFunction() {
    boost::posix_time::milliseconds workTime(10);
    while(true) {
        boost::this_thread::sleep(workTime);

        geometry_msgs::Pose2D pub_pose;
        pub_pose.x = 0.01;
        pub_pose.y = 0.01;
        pub_pose.theta = 0;
        mousePose_pub.publish(pub_pose);
    }
}


void WegTest::sollCallback(const geometry_msgs::Pose2D &soll_data) {

    double speedX=soll_data.x;

    std::stringstream s;
    s << speedX <<"\n";
    std::ofstream outputFile;
    outputFile.open("logSpeed.txt",ios::app);
    //write to file or show here
    outputFile << s.str();
    outputFile.close();



}

