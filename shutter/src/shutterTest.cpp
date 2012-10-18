#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "mobots_msgs/ImageWithPoseAndID.h"

using namespace std;

const char TAG[] = "[shutterTest] ";
sensor_msgs::Image currentImage;
ros::Publisher pub;

void copyMatToImageMSg(const cv::Mat& in, mobots_msgs::ImageWithPoseAndID& out2){
  sensor_msgs::Image* out = &out2.image;
  out->height = in.rows;
  out->width = in.cols;
  out->encoding = "rgb";
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

void* shutterThread(void* data){
  mobots_msgs::ImageWithPoseAndID msg;
  int imageID = 1;
  int mobotID = 0;
  int sessionID = 0;
  while(ros::ok()){
    cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
    FILE* fp;
    char result [1000];
    fp = popen("rospack find slam","r");
    fread(result, 1, sizeof(result), fp);
    pclose(fp);
    stringstream ss;
    result[strlen(result)-1] = '\0';
    ss << result << "/pics/" << imageID << ".png";
    cv::Mat img = cv::imread(ss.str(), 1);
    //copyMatToImageMSg(img, msg);
	 vector<uchar> data;
	 string encoding(".png");
	 imencode(encoding, img, data);
	 msg.image.data = data;
	 msg.image.height = img.rows;
	 msg.image.width = img.cols;
	 msg.image.encoding = "png";
	 msg.image.is_bigendian = 0;
	 msg.image.step = img.cols * img.elemSize();
    msg.pose.x = 0 + imageID*rand()/RAND_MAX*3;
    msg.pose.y = 0 + imageID*rand()/RAND_MAX*3;
    msg.pose.theta = imageID*rand()/RAND_MAX*2;
    pub.publish(msg);
    imageID++;
    msg.id.image_id = imageID;
    cout << TAG << "shuttered  " << ss.str() << endl;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "shutterTest");
  ros::NodeHandle nh;
  pub = nh.advertise<mobots_msgs::ImageWithPoseAndID>("shutter_image_delta_pose", 2);
  pthread_t thread_t;
  pthread_create(&thread_t, 0, shutterThread, 0);
  cout << TAG << "Press [enter] for shutter" << endl;
  ros::spin();
}