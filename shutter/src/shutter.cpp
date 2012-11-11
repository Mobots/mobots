#include "shutter.h"
#include "mobots_common/utils.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "shutter");
	ros::NodeHandle nh;
	int mobotID = 0;
	if(!mobots_common::utils::parseNamespace(nh.getNamespace(), mobotID))
		ROS_ERROR("%s mobotID cannot be parsed from namespace: %s", __PRETTY_FUNCTION__, nh.getNamespace().c_str());

	int method = 0; //0 = poll, 1 = usb_cam
	ros::param::get("/shutter/camera_method", method);
	//Shutter shutter(0,1.06805,0.80104); //l/b für Simulator: 1.06805,0.80104
	//Shutter shutter(mobotID, 3000.06805, 2500.80104);
	if(method == 0){
		Shutter2 shutter(mobotID, 3000.06805, 2500.80104);
		shutter.startShutter();
	}else{
		Shutter shutter(mobotID, 3000.06805, 2500.80104);
		shutter.startShutter();
	}
}

Shutter::Shutter(int mobotID, double l, double b): mobotID(mobotID), g(l,b) //Instanzierung von Geometry
{
		sessionID = 0;
		if(!ros::param::get("/sessionID", sessionID))
		ROS_ERROR("[%s] /sessionID or gtfo, sessionID set to 0", __PRETTY_FUNCTION__);
		ipid.id.session_id = sessionID;
		ipid.id.mobot_id = mobotID;
}

Shutter::~Shutter() {
}

void Shutter::startShutter()
{
    ROS_INFO("[%s] Mobot %d: Shutterfunktion gestartet.", __PRETTY_FUNCTION__, mobotID);
    poseImage_pub = nh.advertise<mobots_msgs::ImageWithPoseAndID>("image_pose_id", 2);

    image_sub = nh.subscribe("usb_cam/image_raw", 5, &Shutter::imageCallback, this);
    pose_sub = nh.subscribe("mouse", 100, &Shutter::mouseCallback, this);

    //ros::ServiceServer service = nh.advertiseService("getDelta", &Shutter::getDelta, this);
    
		overlap = 0.3;
		ros::param::get("/shutter/overlap", overlap);
		
    dX = 0;
    dY = 0;
    dTheta = 0;

    ros::spin();

}

bool Shutter::getDelta(shutter::delta::Request &req, shutter::delta::Response &res)
{
  res.x = dX;
  res.y = dY;
  res.theta = dTheta;
  return true;
}

void Shutter::publishMessage(double x, double y, double theta, const sensor_msgs::Image &image) {
    ipid.image = image;

    geometry_msgs::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;

    ipid.pose = pose;
	 ipid.id.image_id = imageID;
    poseImage_pub.publish(ipid);
	 imageID++;
}

int frame = 0;

void Shutter::imageCallback(const sensor_msgs::Image &mobot_image) {
    double currentOverlap = g.checkPicture(dX, dY, dTheta); //entspricht der derzeitigen überlappung
    frame++;
    if(frame == 25){
      std::cout << "Überlappung: " << currentOverlap << "of " << overlap << "  => " << currentOverlap/overlap*100 << "%" << std::endl;
      frame = 0;
    }
    if (currentOverlap < overlap) {
    std::cout << "shuttered" << std::endl;
        publishMessage(dX, dY, dTheta, mobot_image);
        dX = 0;
        dY = 0;
        dTheta = 0;
    }
}

void Shutter::mouseCallback(const geometry_msgs::Pose2D &mouse_data) {
    dX += mouse_data.x;
    dY += mouse_data.y;
    dTheta += mouse_data.theta;
}
