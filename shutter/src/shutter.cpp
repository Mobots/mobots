#include "shutter.h"

int main(int argc, char** argv){
ros::init(argc, argv, "shutter");
//Shutter shutter(0,1.06805,0.80104); //l/b für Simulator: 1.06805,0.80104
Shutter shutter(0, 3000.06805, 2500.80104);
}
Shutter::Shutter(int mobot_ID,double l, double b):id(mobot_ID),g(l,b) //Instanzierung von Geometry
{
    argc = 0;
    std::stringstream s;
    s << "shutter_" << mobot_ID;
    ros::init(argc, (char**)argv, s.str());
    ros::NodeHandle nh;
    Shutter::startShutter(); 
}

Shutter::~Shutter() {
}

void Shutter::startShutter()
{
    ROS_INFO("Mobot %d: Shutterfunktion gestartet.", id);
    poseImage_pub = nh.advertise<mobots_msgs::ImageWithPoseAndID>("ImageWithPoseAndID", 2);

    image_sub = nh.subscribe("usb_cam/image_raw", 5, &Shutter::imageCallback, this);
    pose_sub = nh.subscribe("mouse/pose", 100, &Shutter::mouseCallback, this);

    //ros::ServiceServer service = nh.advertiseService("mobot_pose/getDelta", &Shutter::getDelta, this);
    
    ros::param::param<double>("overlap",overlap, 0.3);
    dX = 0;
    dY = 0;
    dTheta = 0;

    ros::spin();

}

/*bool Shutter::getDelta(shutter::delta::Request &req, shutter::delta::Response &res)
{
  res.x = dX;
  res.y = dY;
  res.theta = dTheta;
  return true;
}*/

void Shutter::publishMessage(double &x, double &y, double &theta, const sensor_msgs::Image &image) {
    ipid.id.mobot_id = id;
    ipid.image = image;

    geometry_msgs::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;

    ipid.pose = pose;
    poseImage_pub.publish(ipid);
}

int frame = 0;

void Shutter::imageCallback(const sensor_msgs::Image &mobot_image) {
    double safe = g.checkPicture(dX, dY, dTheta);
    frame++;
    if(frame == 25){
      std::cout << "Überlappung: " << safe << "of " << overlap << "  => " << safe/overlap*100 << "%" << std::endl;
      frame = 0;
    }
    if (safe < overlap) {
	std::cout << "shuttered" << std::endl;
        publishMessage(dX, dY, dTheta, mobot_image);
        dX = 0;
        dY = 0;
        dTheta = 0; // evtl. lock machen
    } 
}

void Shutter::mouseCallback(const geometry_msgs::Pose2D &mouse_data) {
    dX += mouse_data.x;
    dY += mouse_data.y;
    dTheta += mouse_data.theta;
}



