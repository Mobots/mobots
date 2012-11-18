#include <iostream>
#include <ros/ros.h>
#include <mobots_msgs/ImageWithPoseAndID.h>
#include <math.h>
#include "geometry.h"
#include <pthread.h>
#include "libusb_cam/usb_cam.h"
#include "mobots_common/utils.h"
#include "sensor_msgs/fill_image.h"

using namespace std;

static const int circle_buffer_count = 3;
static usb_cam_camera_image_t* camera_image_;
static mobots_msgs::ImageWithPoseAndID images_circle_buffer[circle_buffer_count];
static int circle_buffer_index = 0;
static pthread_t cameraThread_t;

static void* cameraThread(void* data);

static bool ok = false;

    ros::Subscriber pose_sub;
    ros::Subscriber image_sub;
    ros::Publisher poseImage_pub;
    
		int sessionID;
		int imageID;
		int mobotID;

		Geometry* g;
		double overlap, dX, dY, dTheta;
		ros::NodeHandle *nh;

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

void startShutter();
static void* cameraThread(void* data);
void startCamera();
void stopCamera();
void mouseCallback2(const geometry_msgs::Pose2D &mouse_data);
void publishMessage(double x, double y, double theta, int index);
static void copyImage();
void handleError(const char* error);


static const int imageWidth = 640;
static const int imageHeight = 480;
 

int main(int argc, char** argv){
	ros::init(argc, argv, "shutter");
	nh = new ros::NodeHandle;
	int mobotID = 0;
	if(!mobots_common::utils::parseNamespace(nh->getNamespace(), mobotID))
		ROS_ERROR("%s mobotID cannot be parsed from namespace: %s", __PRETTY_FUNCTION__, nh->getNamespace().c_str());

	int method = 2; //0 = poll, 1 = usb_cam
	ros::param::get("/shutter/camera_method", method);
	//Shutter shutter(0,1.06805,0.80104); //l/b für Simulator: 1.06805,0.80104
	//Shutter shutter(mobotID, 3000.06805, 2500.80104);
			sessionID = 0;
		if(!ros::param::get("/sessionID", sessionID))
		ROS_ERROR("[%s] /sessionID or gtfo, sessionID set to 0", __PRETTY_FUNCTION__);
		imageID = 0;
		
						overlap = 0.3;
		ros::param::get("/shutter/overlap", overlap);
		//ros::param::get("/shutter/height", imageHeight);
		//ros::param::get("/shutter/width", imageWidth);
		
		for(int i = 0; i < circle_buffer_count; i++){
		  images_circle_buffer[i].image.encoding = string("rgb8");
		  images_circle_buffer[i].image.data.resize(imageWidth*imageHeight*3);
		  images_circle_buffer[i].image.is_bigendian = 0;
		  images_circle_buffer[i].image.width = imageWidth;
		  images_circle_buffer[i].image.height = imageHeight;
		  images_circle_buffer[i].image.step = 3*imageWidth;
		  images_circle_buffer[i].id.mobot_id = mobotID;
		  images_circle_buffer[i].id.session_id = sessionID;
		}
		usb_cam_setErrorHandler(handleError);

	startShutter();
}

void startShutter(){
    ROS_INFO("[%s] Mobot %d: Shutterfunktion gestartet (ultra method).", __PRETTY_FUNCTION__, mobotID);
    poseImage_pub = nh->advertise<mobots_msgs::ImageWithPoseAndID>("image_pose_id", 20);
		
    pose_sub = nh->subscribe("mouse", 10, mouseCallback2);

    //ros::ServiceServer service = nh.advertiseService("getDelta", &Shutter2::getDelta, this);
    
		
    dX = 0;
    dY = 0;
    dTheta = 0;
		g = new Geometry(1,1);
		startCamera();

    ros::spin();

}

void* cameraThread(void* data){
	while(ok){
		usb_cam_camera_grab_image(camera_image_);
		copyImage();
	}
	return 0;
}

static void copyImage(){
  memcpy(&images_circle_buffer[circle_buffer_index].image.data[0], camera_image_->image, imageWidth*imageHeight*3);
  circle_buffer_index++;
  if(circle_buffer_index == circle_buffer_count)
	 cout << "resetting index" << endl;
  circle_buffer_index %= circle_buffer_count;
}

void startCamera(){
	camera_image_ = usb_cam_camera_start("/dev/video0",
		IO_METHOD_MMAP,
		PIXEL_FORMAT_YUYV,
		imageWidth,
		imageHeight);
	ok = true;
	pthread_create(&cameraThread_t, 0, cameraThread, 0);
}

void stopCamera(){
	ok = false;
	pthread_join(cameraThread_t, NULL);
	usb_cam_camera_shutdown();
	free(camera_image_);
}

void publishMessage(double x, double y, double theta, int index) {
  images_circle_buffer[index].pose.x = x;
  images_circle_buffer[index].pose.y = y;
  images_circle_buffer[index].pose.theta = theta;
	images_circle_buffer[index].id.image_id = imageID;
	//deflate();
	poseImage_pub.publish(images_circle_buffer[index]);
	imageID++;
}

void mouseCallback2(const geometry_msgs::Pose2D &mouse_data) {
    dX += mouse_data.x;
    dY += mouse_data.y;
    dTheta += mouse_data.theta;
		
		//callbackCount++;
		if(1/*callbackCount >= 0 || Shutter::MOUSE_THRESHOLD*/){
			//callbackCount = 0;
			double currentOverlap = g->checkPicture(dX, dY, dTheta); //entspricht der derzeitigen überlappung
			std::cout << "dx " << dX << " dy " << dY  << " dTheta " << dTheta << " overlap " << currentOverlap << " need < " << overlap << std::endl;
			if (currentOverlap < overlap){
			  int index = circle_buffer_index-2;
			  if(index >= circle_buffer_count) //possible because we don't use mutex
				 index = circle_buffer_count-1;
				if(index < 0)
					index = circle_buffer_count-1;
				//msg.image.data = camera_circle_buffer[index].image;
				std::cout << __FILE__ << "shuttering "  << endl;
				publishMessage(dX, dY, dTheta, index);
				dX = 0;
				dY = 0;
				dTheta = 0;
			}
		}
}

void handleError(const char* error){
	//like we give a fuck
	//just restart the damn camera
	stopCamera();
	usleep(100);
	startCamera();
	cout << "restarting camera" << endl;
}