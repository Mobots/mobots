#include "mobots_msgs/CvMat.h"
#include "mobots_msgs/CvKeyPoint.h"
#include "feature_detector/MessageBridge.h"

using namespace std;
using namespace cv;

void MessageBridge::copyToRosMessage(const FeatureSet& in, mobots_msgs::FeatureSet& out){
  const int N = in.keyPoints.size();
  out.keyPoints.resize(N);

  for(int i = 0; i < N; i++){
    mobots_msgs::CvKeyPoint& outKeyPoint = out.keyPoints[i];
    const KeyPoint& inKeyPoint = in.keyPoints[i];
    outKeyPoint.pt.x = inKeyPoint.pt.x;
    outKeyPoint.pt.y = inKeyPoint.pt.y;
    outKeyPoint.size = inKeyPoint.size;
    outKeyPoint.angle = inKeyPoint.angle;
    outKeyPoint.response = inKeyPoint.response;
    outKeyPoint.octave = inKeyPoint.octave;
    outKeyPoint.class_id = inKeyPoint.class_id;
  }
  
  mobots_msgs::CvMat& descriptors = out.descriptors; //alias
  
  //from cv_bridge
  descriptors.rows = in.descriptors.rows;
  descriptors.cols = in.descriptors.cols;
  descriptors.type = in.descriptors.type();
  descriptors.elemSize = in.descriptors.elemSize();
  int step = in.descriptors.cols * in.descriptors.elemSize();
  descriptors.data.resize(in.descriptors.rows * step);
  if(in.descriptors.isContinuous()){
    memcpy(&descriptors.data[0], in.descriptors.data, in.descriptors.rows * step);
    //cout << "copyToRos continous" << endl;
  }else{
    // Copy row by row
    uchar* ros_data_ptr = (uchar*)(&descriptors.data[0]);
    uchar* cv_data_ptr = in.descriptors.data;
    for (int i = 0; i < in.descriptors.rows; i++){
      memcpy(ros_data_ptr, cv_data_ptr, step);
      ros_data_ptr += step;
      cv_data_ptr += in.descriptors.step;
    }
    //cout << "copyToRos not continous" << endl;
  }
}

void MessageBridge::copyToCvStruct(const mobots_msgs::FeatureSet& in, FeatureSet& out){
  const int N = in.keyPoints.size();
  out.keyPoints.resize(N);
  
  for(int i = 0; i < N; i++){
    KeyPoint& outFeaturePoint = out.keyPoints[i];
    const mobots_msgs::CvKeyPoint& inFeaturePoint = in.keyPoints[i];
    outFeaturePoint.pt.x = inFeaturePoint.pt.x;
    outFeaturePoint.pt.y = inFeaturePoint.pt.y;
    outFeaturePoint.size = inFeaturePoint.size;
    outFeaturePoint.angle = inFeaturePoint.angle;
    outFeaturePoint.response = inFeaturePoint.response;
    outFeaturePoint.octave = inFeaturePoint.octave;
    outFeaturePoint.class_id = inFeaturePoint.class_id;
  }
  
  const mobots_msgs::CvMat& inDescriptors = in.descriptors; //alias
  out.descriptors.create(inDescriptors.rows, inDescriptors.cols, inDescriptors.type);
  memcpy(out.descriptors.data, &inDescriptors.data[0], 
	 inDescriptors.rows * inDescriptors.cols * inDescriptors.elemSize);
}
