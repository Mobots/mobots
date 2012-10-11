#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <boost/lexical_cast.hpp>

#include "feature_detector/FeaturesFinder.h"
#include "feature_detector/MessageBridge.h"
#include "feature_detector/FeaturesMatcher.h"
#include "mobots_msgs/ImageWithPoseAndID.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"



using namespace cv;
using namespace std;


void copyMatToImageMSg(const cv::Mat& in, mobots_msgs::ImageWithPoseAndID& out2){
  sensor_msgs::Image* out = &out2.image;
  out->height = in.rows;
  out->width = in.cols;
  out->encoding = "mono8";
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
  ros::init(argc, argv, "testFeatures");

  ros::NodeHandle nodeHandle;
  ros::Publisher publisher = nodeHandle.advertise<mobots_msgs::ImageWithPoseAndID>("image_pose_id", 2);
  
  for (int i = 1; i <= 13; i++)
  {
    Mat image = imread("/home/mobots/ros_workspace/mobots/slam/pics/" + boost::lexical_cast<string>(i) + ".png", 1); //1 for colours
    Mat gray_image;
    cvtColor(image, gray_image, CV_RGB2GRAY); //FeatureDetecter etc. arbeiten alle auf Graustufenbildern
    
    mobots_msgs::ImageWithPoseAndID mobot_image;
    copyMatToImageMSg(gray_image, mobot_image);
    mobot_image.id.session_id = 0;
    mobot_image.id.mobot_id = 1;
    mobot_image.id.image_id = i;
    mobot_image.pose.x = 10*i;
    mobot_image.pose.y = 10*i;
    mobot_image.pose.theta = 0;
    
    cout << "Press [Enter] for publishing " << i << ".png" << endl;
    cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
    
    publisher.publish(mobot_image);
  }
}