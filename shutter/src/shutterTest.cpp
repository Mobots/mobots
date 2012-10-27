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
  int imageID[4] = {0};
  int sessionID = 0;
	int mobotCount = 3;
  while(ros::ok()){
    cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
		for(int i = 1; i <= mobotCount; i++){
			int mobotID = i;
			FILE* fp;
			char result [1000];
			fp = popen("rospack find slam","r");
			fread(result, 1, sizeof(result), fp);
			pclose(fp);
			stringstream ss;
			result[strlen(result)-1] = '\0';
			int pic = ((double)rand())/((double)RAND_MAX)*18 +1;
			ss << result << "/pics/karte/" << pic << ".png";
			cout << TAG << "mobot" << mobotID << "shuttered  " << ss.str() << endl;
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
			msg.pose.x = -0.3 + pic*(double)((double)rand()/(double)RAND_MAX)*3;
			msg.pose.y = 0.1 + pic*(double)((double)rand()/(double)RAND_MAX)*3;
			msg.pose.theta = pic*(double)((double)rand()/(double)RAND_MAX)*2*M_PI;
			pub.publish(msg);
			msg.id.image_id = imageID[i];
			msg.id.mobot_id = mobotID;
			imageID[i]++;
			cout << "x " << msg.pose.x << " y " << msg.pose.y << " theta " << msg.pose.theta << endl;
		}
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "shutterTest");
  ros::NodeHandle nh;
  pub = nh.advertise<mobots_msgs::ImageWithPoseAndID>("/mobot1/image_pose_id", 2);
  pthread_t thread_t;
  pthread_create(&thread_t, 0, shutterThread, 0);
  cout << TAG << "Press [enter] for shutter" << endl;
  ros::spin();
}