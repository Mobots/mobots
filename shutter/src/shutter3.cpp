#include "shutter.h"
#include <iostream>
#include <pthread.h>

using namespace std;

static usb_cam_camera_image_t* camera_image_;
static pthread_t cameraThread;

static void* cameraThread(void* data);

static bool ok = false;

Shutter3::Shutter3(int mobotID, double l, double b): Shutter(mobotID, l, b){
}

Shutter3::~Shutter3(){
}

void Shutter3::startShutter(){
    ROS_INFO("[%s] Mobot %d: Shutterfunktion gestartet (ultra method).", __PRETTY_FUNCTION__, mobotID);
    poseImage_pub = nh.advertise<mobots_msgs::ImageWithPoseAndID>("image_pose_id", 20);
		
    pose_sub = nh.subscribe("mouse", 10, &Shutter3::mouseCallback, this);

    //ros::ServiceServer service = nh.advertiseService("getDelta", &Shutter2::getDelta, this);
    
		overlap = 0.3;
		ros::param::get("/shutter/overlap", overlap);
		ros::param::get("/shutter/height", imageHeight);
		ros::param::get("/shutter/width", imageWidth);
		
    dX = 0;
    dY = 0;
    dTheta = 0;
		
		usb_cam_setErrorHandler(this);
		
		startCamera();

    ros::spin();

}

static void* cameraThread(void* data){
	while(ok){
		usb_cam_camera_grab_image(camera_image_);
	}
	return 0;
}

void Shutter3::startCamera(){
	camera_image_ = usb_cam_camera_start("/dev/video0",
		IO_METHOD_MMAP,
		PIXEL_FORMAT_YUYV,
		imageWidth,
		imageHeight);
	ok = true;
	pthread_create(&cameraThread, 0, cameraThread, 0);
}

void Shutter3::stopCamera(){
	ok = false;
	pthread_join(&cameraThread, NULL);
	usb_cam_camera_shutdown();
}

inline void Shutter3::publishMessage(double x, double y, double theta) {
	geometry_msgs::Pose2D pose;
	pose.x = x;
	pose.y = y;
	pose.theta = theta;

	ipid.pose = pose;
	ipid.id.image_id = imageID;
	//deflate();
	poseImage_pub.publish(ipid);
	imageID++;
}

void Shutter3::mouseCallback(const geometry_msgs::Pose2D &mouse_data) {
    dX += mouse_data.x;
    dY += mouse_data.y;
    dTheta += mouse_data.theta;
		
		callbackCount++;
		if(callbackCount >= Shutter::MOUSE_THRESHOLD){
			callbackCount = 0;
			double currentOverlap = g.checkPicture(dX, dY, dTheta); //entspricht der derzeitigen überlappung
			std::cout << "dx " << dX << " dy " << dY  << " dTheta " << dTheta << " overlap " << currentOverlap << " need < " << overlap << std::endl;
			if (currentOverlap < overlap) {
				std::cout << __FILE__ << "shuttering" << std::endl;
				fillImage(ipid.image, "rgb8", camera_image_->height, camera_image_->width, 3 * camera_image_->width, camera_image_->image);
				publishMessage(dX, dY, dTheta);
				dX = 0;
				dY = 0;
				dTheta = 0;
			}
		}
}

void Shutter3::handleError(const char* error){
	//like we give a fuck
	//just restart the damn camera
	stopCamera();
	startCamera();
	cout << "restarting camera" << endl;
}