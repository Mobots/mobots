#include <cstdio>
#include <../../opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/include/ros/ros.h>
#include <shutter/ImagePoseID.h>
#include <math.h>

class Shutter {


public:
    Shutter(int mobot_ID);
    ~Shutter();
    void imageCallback(const sensor_msgs::Image &mobot_image);
    void mouseCallback(const geometry_msgs::Pose2D &mouse_data);
    void startShutter();
    void publishMessage(double& x, double& y, double& theta, const sensor_msgs::Image& image);
    double checkPicture(double x, double y, double theta);

    ros::Subscriber pose_sub;
    ros::Subscriber image_sub;
    ros::Publisher poseImage_pub;
    shutter::ImagePoseID ipid;
    int id;
    double overlap;
    double dX, dY, dTheta, globalX, globalY, globalTheta;
    int argc;
    char argv;
    ros::NodeHandle nh;

};
