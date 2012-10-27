#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "mobots_msgs/ImageWithPoseAndID.h"
#include <mobots_msgs/Pose2DPrio.h>
#include <geometry_msgs/Pose2D.h>

using namespace std;

const char TAG[] = "[guiTest] ";
sensor_msgs::Image currentImage;
ros::Publisher imagePubs[4];
ros::Subscriber nextPoseSubRel[4];
ros::Subscriber nextPoseSubAbs[4];
ros::Publisher globalPosePubs[4];
geometry_msgs::Pose2D globalPoses[4];

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
			int sign = ((double)rand())/((double)RAND_MAX) > 0.5;
			bool usePose = ((double)rand())/((double)RAND_MAX) > 0.5;
			if(usePose){
				msg.pose.x = globalPoses[mobotID].x;
				msg.pose.y = globalPoses[mobotID].y;
				msg.pose.theta = globalPoses[mobotID].theta;
			}else{
				msg.pose.x = sign*pic*(double)((double)rand()/(double)RAND_MAX)*3;
				msg.pose.y = sign*pic*(double)((double)rand()/(double)RAND_MAX)*3;
				msg.pose.theta = sign*pic*(double)((double)rand()/(double)RAND_MAX)*2*M_PI;
			}
			imagePubs[i].publish(msg);
			msg.id.image_id = imageID[i];
			msg.id.mobot_id = mobotID;
			msg.id.session_id = sessionID;
			imageID[i]++;
			cout << "x " << msg.pose.x << " y " << msg.pose.y << " theta " << msg.pose.theta << endl;
		}
  }
}

void relPoseCallback(int mobotID, const mobots_msgs::Pose2DPrio& deltaPose){
	double cost = cos(globalPoses[mobotID].theta);
	double sint = sin(globalPoses[mobotID].theta);
	globalPoses[mobotID].x += cost*deltaPose.pose.x - sint*deltaPose.pose.y;
	globalPoses[mobotID].y += sint*deltaPose.pose.x + cost*deltaPose.pose.y;
	globalPoses[mobotID].theta += deltaPose.pose.theta;
	
	globalPosePubs[mobotID].publish(globalPoses[mobotID]);
}

void absPoseCallback(int mobotID, const mobots_msgs::Pose2DPrio& absPose){
	globalPoses[mobotID] = absPose.pose;
	
	globalPosePubs[mobotID].publish(globalPoses[mobotID]);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "shutterTest");
  ros::NodeHandle nh;
	for(int i = 0; i < 4; i++){
		std::stringstream ss;
		ss << "/mobot" << i << "/image_pose_id";
		imagePubs[i] = nh.advertise<mobots_msgs::ImageWithPoseAndID>(ss.str(), 2);
		ss << "/mobot" << i << "waypoint_abs";
		
		ss.str( std::string() );
		ss.clear();
		ss << "/mobot" << i << "/waypoint_rel";
		boost::function<void(mobots_msgs::Pose2DPrio)> f = boost::bind(relPoseCallback, i, _1);
		nextPoseSubRel[i] = nh.subscribe<mobots_msgs::Pose2DPrio>(ss.str(), 5, f);
		
		ss.str( std::string() );
		ss.clear();
		ss << "/mobot" << i << "/waypoint_abs";
		boost::function<void(mobots_msgs::Pose2DPrio)> f2 = boost::bind(relPoseCallback, i, _1);
		nextPoseSubAbs[i] = nh.subscribe<mobots_msgs::Pose2DPrio>(ss.str(), 5, f2);
		
		ss.str( std::string() );
		ss.clear();
		ss << "/mobot" << i << "/pose";
		globalPosePubs[i] = nh.advertise<geometry_msgs::Pose2D>(ss.str(), 5);
	}

  pthread_t thread_t;
  pthread_create(&thread_t, 0, shutterThread, 0);
  cout << TAG << "Press [enter] for shutter" << endl;
  ros::spin();
}