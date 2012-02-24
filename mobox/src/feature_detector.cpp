#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <dynamic_reconfigure/server.h>
#include <mobox/DetectorConfig.h>

using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr&);
void configureCallback(const mobox::DetectorConfig&, uint32_t);

image_transport::Subscriber image_sub;
image_transport::Publisher image_pub;
FeatureDetector *detector;
int frameCount;
int msecs;

int main(int argc, char **argv) {
  ros::init(argc, argv, "feature_detector");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  image_sub = it.subscribe("/usb_cam/image_raw", 10, imageCallback);
  image_pub = it.advertise("/my_cam/featured", 10);
  
  dynamic_reconfigure::Server<mobox::DetectorConfig> server;
  server.setCallback(configureCallback);
  detector = new OrbFeatureDetector;

  ros::spin();
}

void configureCallback(const mobox::DetectorConfig &newConfig, uint32_t level){
  delete detector;
  switch(newConfig.detector){
    case 0: detector = new FastFeatureDetector; break;
    case 1: detector = new StarFeatureDetector; break;
    case 2: detector = new SiftFeatureDetector; break;
    case 3: detector = new SurfFeatureDetector; break;
    case 4: detector = new OrbFeatureDetector; break;
    case 5: detector = new MserFeatureDetector; break;
    case 6: detector = new GoodFeaturesToTrackDetector; break;
    case 7: detector = new DenseFeatureDetector; break;
    case 8: detector = new SimpleBlobDetector; break;
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_bridge::CvImagePtr p2;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    p2 = cv_bridge::toCvCopy(msg);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  std::vector<KeyPoint> keypoints;
  double time = (double)getTickCount();
  detector->detect(cv_ptr->image, keypoints);
  time = ((double)getTickCount() - time)*1000/getTickFrequency();
  drawKeypoints(p2->image, keypoints, p2->image, Scalar(50, 50));
  image_pub.publish(p2->toImageMsg());
  
  msecs += time;
  frameCount++;
  if(msecs >= 1000){
    ROS_INFO("Currently processing %i images per second", frameCount);
    msecs = 0;
    frameCount = 0;    
  }
}
