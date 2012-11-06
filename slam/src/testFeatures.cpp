#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
//#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <stdlib.h>
#include <ros/package.h>

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
  out->encoding = "bgr8";
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
  
  for (int i = 1; i <= 19; i++)
  {
    string slam_path = ros::package::getPath("slam");
    string filename = slam_path + "/pics/karte/" + boost::lexical_cast<string>(i) + ".png";
    Mat image = imread(filename, 1); //1 for colours
    if (image.data == NULL) {
      cerr << "Error loading pic " << filename << "!" << endl;
      return EXIT_FAILURE;
    }
    //Mat gray_image;
    //cvtColor(image, gray_image, CV_RGB2GRAY); //FeatureDetecter etc. arbeiten alle auf Graustufenbildern

    cv::namedWindow("window", 1);
    cv::imshow("window", image);
    
    mobots_msgs::ImageWithPoseAndID mobot_image;
    copyMatToImageMSg(image, mobot_image);
    mobot_image.id.session_id = 0;
    mobot_image.id.mobot_id = 1;
    mobot_image.id.image_id = i-1;
    mobot_image.pose.x = 0.05;
    mobot_image.pose.y = 0.05;
    mobot_image.pose.theta = 0;
    
    cout << "Press [Enter] for publishing " << i << ".png" << endl;
    cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
    
    publisher.publish(mobot_image);
  }
}
