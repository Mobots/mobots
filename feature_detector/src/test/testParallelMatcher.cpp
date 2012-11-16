#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include "mobots_msgs/FeatureSetWithPoseAndID.h"
#include "mobots_msgs/ImageWithPoseAndID.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "feature_detector/MessageBridge.h"
#include "feature_detector/ParallelMatcher.h"

using namespace std;

class Impl;

ros::NodeHandle* nh;
int picCount = 19;
int threadCount = 1;
vector<FeatureSet*> featureSets;
Impl* impl;
ParallelMatcher* awesomeMatcher;
double startTime;

int resultcount = 0;

class Impl : public ParallelMatcher::ResultListener{
  void onResult(MatchResult result, bool matchable, int id1, int id2){
	 cout << "onResult for id " << id1 << "-" << id2 << ": " << (matchable ? "matchable" : "not matchable") << endl;
	 resultcount++;
	 if(resultcount == picCount*picCount){
		double execTime = ((double)cv::getTickCount() - startTime)/cv::getTickFrequency();
		std::cout << "ParallelMatcher with threadcount " << threadCount <<  ": " << execTime << "s" << std::endl;
		sleep(1);
		exit(0);
	 }
  }
  
  void onLowWatermark(){
	 //cout << "onLowWatermark" << endl;
  }
  void onHighWatermark(){
	 //cout << "onHighWatermark" << endl;
  }
};

int ftrCnt = 0;
void featureHandler(const mobots_msgs::FeatureSetWithPoseAndID& msg){
  FeatureSet set;
  MessageBridge::copyToCvStruct(msg.features, set);
  FeatureSet* set2 = new FeatureSet(set);
  featureSets.push_back(set2);
  ftrCnt++;
  if(ftrCnt < picCount)
	 return;
  
  cout << "received all featuresets" << endl;

  startTime = (double)cv::getTickCount();
  for(int i = 0; i < picCount; i++){
	 for(int i2 = 0; i2 < picCount; i2++){
		//bruteforce
		awesomeMatcher->enqueue(featureSets.at(i), featureSets.at(i2), i, i2);
	 }
  }
}

void copyMatToImageMSg(const cv::Mat& in, mobots_msgs::ImageWithPoseAndID& out2){
  sensor_msgs::Image* out = &out2.image;
  out->height = in.rows;
  out->width = in.cols;
  out->encoding = "rgb8";
  out->is_bigendian = 0;
  out->step = in.cols * in.elemSize();
  out->data.resize(in.rows * out->step);
  if(in.isContinuous()){
    memcpy(&out->data[0], in.data, in.rows * out->step);
  }else{
    // Copy row by row
    uchar* ros_data_ptr = (uchar*)(&out->data[0]);
    uchar* cv_data_ptr = in.data;
    for (int i = 0; i < in.rows; i++){
      memcpy(ros_data_ptr, cv_data_ptr, out->step);
      ros_data_ptr += out->step;
      cv_data_ptr += in.step;
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "testParallelMatcher");
    threadCount = atoi(argv[1]);
  nh = new ros::NodeHandle;
  string picFolder = ros::package::getPath("slam") + string("/pics/karte/");
  ros::Subscriber sub = nh->subscribe("/mobot1/featureset_pose_id", 5, featureHandler);
  ros::Publisher pub = nh->advertise<mobots_msgs::ImageWithPoseAndID>("/mobot1/image_pose_id", 5);
  sleep(1);
    impl = new Impl;
  awesomeMatcher = new ParallelMatcher(impl, threadCount, 5, 10);
  for(int i = 0; i < picCount; i++){
	 mobots_msgs::ImageWithPoseAndID msg;
	 stringstream ss;
	 ss << i;
	 cv::Mat img = cv::imread(picFolder+ss.str()+string(".png"), 1);
	 copyMatToImageMSg(img, msg);
	 pub.publish(msg);
	 cout << "." << flush;
	 sleep(1);
	 ros::spinOnce();
  }
  ros::spin();
}