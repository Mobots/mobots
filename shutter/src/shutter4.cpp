#include <iostream>
#include <ros/ros.h>
#include <mobots_msgs/ImageWithPoseAndID.h>
#include "geometry.h"
#include "libusb_cam/usb_cam.h"
#include "mobots_common/utils.h"
#include "signal.h"

using namespace std;

usb_cam_camera_image_t* camera_image_;
mobots_msgs::ImageWithPoseAndID msg;

bool restartNeeded = false;
bool initialized = false;

ros::Subscriber pose_sub;
ros::Subscriber image_sub;
ros::Publisher poseImage_pub;

int sessionID;
int imageID;
int mobotID;

Geometry* g;
double overlap, dX, dY, dTheta;
ros::NodeHandle *nh;

void startShutter();
void cameraThread();
void startCamera();
void stopCamera();
void mouseCallback(const geometry_msgs::Pose2D &mouse_data);
void publishMessage();
void copyImage();
void handleError(const char* error);
void sigHandler(int signum);
void checkOverlap();


const int imageWidth = 640;
const int imageHeight = 480;
 

int main(int argc, char** argv){
	ros::init(argc, argv, "shutter");
	nh = new ros::NodeHandle;
	int mobotID = 0;
	if(!mobots_common::utils::parseNamespace(nh->getNamespace(), mobotID))
		ROS_ERROR("%s mobotID cannot be parsed from namespace: %s", __PRETTY_FUNCTION__, nh->getNamespace().c_str());
	
	//Shutter shutter(0,1.06805,0.80104); //l/b für Simulator: 1.06805,0.80104
	//Shutter shutter(mobotID, 3000.06805, 2500.80104);
			sessionID = 0;
		if(!ros::param::get("/sessionID", sessionID))
		  ROS_ERROR("[%s] /sessionID or gtfo, sessionID set to 0", __PRETTY_FUNCTION__);
		
						overlap = 0.3;
		ros::param::get("/shutter/overlap", overlap);
		//ros::param::get("/shutter/height", imageHeight);
		//ros::param::get("/shutter/width", imageWidth);
		  msg.image.encoding = string("rgb8");
		  msg.image.data.resize(imageWidth*imageHeight*3);
		  msg.image.is_bigendian = 0;
		  msg.image.width = imageWidth;
		  msg.image.height = imageHeight;
		  msg.image.step = 3*imageWidth;
		  msg.id.mobot_id = mobotID;
		  msg.id.session_id = sessionID;
		usb_cam_setErrorHandler(handleError);
	 signal(SIGINT, sigHandler);
	startShutter();
}

void sigHandler(int signum){
  usb_cam_camera_shutdown();
  exit(0);
}

void startShutter(){
  ROS_INFO("[%s] Mobot %d: Shutterfunktion gestartet (ultra method444).", __PRETTY_FUNCTION__, mobotID);
  poseImage_pub = nh->advertise<mobots_msgs::ImageWithPoseAndID>("image_pose_id", 20);
	 
  pose_sub = nh->subscribe("mouse", 1000, mouseCallback);
	 
  imageID = 0;
  dX = 0;
  dY = 0;
  dTheta = 0;
  g = new Geometry(1,1);
	 
  startCamera();
  cameraThread(); //endless (summer) loop
}

void cameraThread(){
	while(1){
	 usb_cam_camera_grab_image(camera_image_);
	 if(restartNeeded){
		stopCamera();
		startCamera();
	  }
	 ros::spinOnce();
	 checkOverlap();
	}
}

void startCamera(){
	camera_image_ = usb_cam_camera_start("/dev/video0",
		IO_METHOD_MMAP,
		PIXEL_FORMAT_YUYV,
		imageWidth,
		imageHeight);
	initialized = true;
}

void stopCamera(){
	usb_cam_camera_shutdown();
	free(camera_image_);
	initialized = false;
}

 void copyImage(){
  memcpy(&msg.image.data[0], camera_image_->image, imageWidth*imageHeight*3);
}

void publishMessage() {
  copyImage();
  msg.pose.x = dX;
  msg.pose.y = dY;
  msg.pose.theta = dTheta;
  msg.id.image_id = imageID;
  //deflate(); //need opencv to compress to png
  poseImage_pub.publish(msg);
  imageID++;
}

void checkOverlap(){
  double currentOverlap = g->checkPicture(dX, dY, dTheta); //entspricht der derzeitigen überlappung
  cout << "dx " << dX << " dy " << dY  << " dTheta " << dTheta << " overlap ";
  cout << currentOverlap << " need < " << overlap << endl;
  if (currentOverlap < overlap){
	 std::cout << __FILE__ << "shuttering "  << endl;
	 publishMessage();
	 dX = 0;
	 dY = 0;
	 dTheta = 0;
  }
}

void mouseCallback(const geometry_msgs::Pose2D &mouse_data) {
  dX += mouse_data.x;
  dY += mouse_data.y;
  dTheta += mouse_data.theta;
}

void handleError(const char* error){
	//like we give a fuck
	//just restart the damn camera
	if(!initialized){
	 cout << "error while initializing, please restart this shit by hand" << endl;
	 exit(1);
	}
	restartNeeded = true;
	cout << "restarting camera" << endl;
}