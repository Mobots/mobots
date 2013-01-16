#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <iostream>
#include <stdlib.h>
#include <ros/package.h>
#include "signal.h"

#include "mobots_msgs/ImageWithPoseAndID.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"


/**
 * This program is used to send mock images with custom delta values
 */

using namespace cv;
using namespace std;

void sigHandler(int signum){
  cout << endl;
  exit(0);
}

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
  ros::init(argc, argv, "mockImage");

  ros::NodeHandle nodeHandle;
  ros::Publisher pub = nodeHandle.advertise<mobots_msgs::ImageWithPoseAndID>("/mobot0/image_pose_id", 2);
  double x, y, theta;
  string slam_path = ros::package::getPath("slam");
  signal(SIGINT, sigHandler);
  cout << "input delta values in CENTI meters" << endl;
  for (int i = 0, id = 0; true; i++, id++, i %= 25){
	 stringstream ss;
	 ss << slam_path << "/pics/karte2/" << i << ".png";
	 string filename = ss.str();
    Mat image = imread(filename, 1); //1 for colours
    if (image.data == NULL){
      cerr << "Error loading pic " << filename << "!" << endl;
      return EXIT_FAILURE;
    }
    //Mat gray_image;
    //cvtColor(image, gray_image, CV_RGB2GRAY); //FeatureDetecter etc. arbeiten alle auf Graustufenbildern
	 cout << "x: " << flush;
	 cin >> x;
	 cout << "y: " << flush;
	 cin >> y;
	 cout << "theta: " << flush;
	 cin >> theta;
	 cout << "new position: " << x << ", " <<
		y << ", " << theta << endl << endl;
	 
    mobots_msgs::ImageWithPoseAndID mobot_image;
    copyMatToImageMSg(image, mobot_image);
    mobot_image.id.session_id = 0;
    mobot_image.id.mobot_id = 0;
    mobot_image.id.image_id = id;
    mobot_image.pose.x = x/100;
    mobot_image.pose.y = y/100;
    mobot_image.pose.theta = theta;
	 cout << "send pic " << id << "with delta: " << x << ", " <<
		y << ", " << theta << endl << endl;
    pub.publish(mobot_image);
  }
}
