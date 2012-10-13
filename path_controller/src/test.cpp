#include "weg_test.h"

int main(int argc, char** argv)
{
ros::init(argc, argv, "wegTest");
WegTest startTheFuckingTest;
}

WegTest::WegTest() 			//Konstruktor Weg
{
  argc = 0;
  std::stringstream s;
  //ros::init(argc, (char**)argv, s.str());
  startWegTest();
}

WegTest::~WegTest()				//Destruktor
{
}

void WegTest::startWegTest()
{
    //subscribes to
    ros::NodeHandle nh;
    sollV_sub = nh.subscribe("driver/sollV", 30, &WegTest::sollCallback, this);

    mousePose_pub = nh.advertise<geometry_msgs::Pose2D>("mouse/pose", 5);
    targetPose_pub = nh.advertise<mobots_msgs::Pose2DPrio>("mobot_pose/waypoint", 2);

    //set target to get to
    mobots_msgs::Pose2DPrio pub_pose;
    pub_pose.pose.x = 5;
    pub_pose.pose.y = 6;
    pub_pose.pose.theta = 0;
    pub_pose.prio = 0;
	 sleep(2); //ansonsten sind die nodes noch nicht verbunden
    targetPose_pub.publish(pub_pose);

    //start process to publish mousedata with 100hz
	 pthread_t thread_t;
	 pthread_create(&thread_t, 0, threadHelper, this);
    //boost::thread pubThread(boost::bind(&WegTest::pubFunction, this));
    //pubThread.join();

    ros::spin();
}

void WegTest::pubFunction() {
  ros::Rate rate(1/(30*0.001)); 
    //boost::posix_time::milliseconds workTime(10);
    while(true) {
        //boost::this_thread::sleep(workTime);

        geometry_msgs::Pose2D pub_pose;
        pub_pose.x = 0.01;
        pub_pose.y = 0.01;
        pub_pose.theta = 0;
        mousePose_pub.publish(pub_pose);
		  rate.sleep();
    }
}

void* threadHelper(void* object){
  ((WegTest*)object)->pubFunction();
}


void WegTest::sollCallback(const geometry_msgs::Pose2D &soll_data) {

    double speedX=soll_data.x;
    std::stringstream s;
    s << speedX <<"\n";
    std::ofstream outputFile;
    outputFile.open("logSpeed.txt", std::ios::app);
    //write to file or show here
    outputFile << s.str();
    outputFile.close();



}

