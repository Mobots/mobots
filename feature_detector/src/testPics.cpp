#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "feature_detector/FeaturesFinder.h"
#include "feature_detector/MessageBridge.h"
#include "feature_detector/FeaturesMatcher.h"
#include "mobots_msgs/ImageWithPoseAndID.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"



using namespace cv;
using namespace std;

bool done = false;
bool matchable = false;
int int1;
int int2;

void featuresReceived(const mobots_msgs::FeatureSetWithPoseAndID& featuresMsg);

void* systemThreaded(void* data){
	system((char*)data);
	delete[] data;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "schnauze");
  int picCount = 26;
  ros::NodeHandle nodeHandle;
	FILE* pf = fopen("/dev/null", "r"); //stdout stfu
  *stdout = *pf;
  string base = string("/home/jonas/mobots/slam") + string("/pics/karte2/");
  for(int i = 0; i < picCount; i++){
		for(int i2 = i+1; i2 < picCount; i2++){

			int1 = i;
			int2 = i2;
		  stringstream ss1;
		  ss1 << base << i << ".png";
		  stringstream ss2;
		  ss2 << base << i2 << ".png";
			stringstream ss;
			ss << "rosrun feature_detector makeMatch " << ss1.str() << " " << ss2.str() << " " << i << " " << i2;
			char* meh = new char[ss.str().size()+5];
			strcpy(meh, ss.str().c_str());
			pthread_t thread;
			//pthread_create(&thread, 0, systemThreaded, meh);
			system(meh);
			cerr << ss.str() << endl;
		  /*char** paths = (char**) malloc(3*sizeof(char*));
		  char* path0 = (char*) calloc(1000, 1);
		  char* path1 = (char*) calloc(1000, 1);
		  char* path2 = (char*) calloc(1000, 1);
		  strcpy(path0, "schnauze");
		  strcpy(path1, ss1.str().c_str());
		  strcpy(path2, ss2.str().c_str());
		  paths[0] = path0;
		  paths[1] = path1;
		  paths[2] = path2;
		  main2(3, paths );
		  while(!done){
			 ros::spinOnce();
			 sleep(1);
		  }
		  done = false;
		  /*if(matchable)
			 cerr << i << ".png and " << i2 << ".png are matchable" << endl;
		  else if(abs(i-i2) <= 2)
			  cerr << i << ".png and " << i2 << ".png are NOT matchable" << endl;*/
		}
  }
  exit(1);
}