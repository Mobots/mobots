#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "feature_detector/FeaturesFinder.h"
#include "feature_detector/MessageBridge.h"
#include "FeaturesMatcher.h"
#include "mobots_msgs/FeatureSetWithDeltaPose.h"
#include "mobots_msgs/ImageWithPoseDebug.h"
#include "std_msgs/String.h"



using namespace cv;
using namespace std;

Mat gimage1; //for debug
Mat gimage2;
extern Mat H;

char pos = 0;
ImageFeatures features1;
ImageFeatures features2;

/**
 * angle is in radian kk
 */
void findRotationMatrix2D(Point2f center, double angle, Mat& rotMat){
    double alpha = cos(angle);
    double beta = sin(angle);
    rotMat.create(2, 3, CV_64F);
    double* m = (double*)rotMat.data;

    m[0] = alpha;
    m[1] = beta;
    m[2] = (1-alpha)*center.x - beta*center.y;
    m[3] = -beta;
    m[4] = alpha;
    m[5] = beta*center.x + (1-alpha)*center.y;
}

void copyMatToImageMSg(const cv::Mat& in, mobots_msgs::ImageWithPoseDebug& out2){
  sensor_msgs::Image* out = &out2.image;
  out->height = in.rows;
  out->width = in.cols;
  out->encoding = "mono8";
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


inline double toDegree(double rad){
  return rad * 180 / M_PI;
}

void checkResult(){
  Delta delta;
  Ptr<FeaturesMatcher> matcher = new CpuFeaturesMatcher(CpuFeaturesMatcher::ORB_DEFAULT);
  bool matchResult = matcher->match(features1, features2, delta);
  if(!matchResult){
    cout << "images do not overlap at all" << endl;
    return;
  }
  cout << "deltaX " << delta.x << endl;
  cout << "deltaY " << delta.y << endl;
  cout << "theta " << delta.theta << " rad = " << toDegree(delta.theta) << "°" << endl;
  Mat aff;
  findRotationMatrix2D(Point2f(gimage2.cols/2, gimage2.rows/2), delta.theta, aff);
  aff.at<double>(0,2) = delta.x;
  aff.at<double>(1,2) = delta.y;
  cout << "affen mat: " << endl << aff << endl;
  Mat result;
  Mat result2;
  result2.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  result.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  Mat outImg1 = result(Rect(0, 0, gimage1.cols, gimage1.rows));
  Mat outImg21 = result2(Rect(0, 0, gimage1.cols, gimage1.rows));

  warpAffine(gimage2, result, aff, result.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  gimage1.copyTo(outImg1);
  
  gimage1.copyTo(outImg21);
  warpAffine(gimage2, result2, aff, result.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  
  imshow("result", result);
  imshow("result2", result2);
  
  cout << "H mat " << endl << H << endl;
  
  Mat result3;
  Mat result4;
  result3.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  result4.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  Mat outImg3 = result3(Rect(0, 0, gimage1.cols, gimage1.rows));
  Mat outImg41 = result4(Rect(0, 0, gimage1.cols, gimage1.rows));

  warpPerspective(gimage2, result3, H, result3.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  gimage1.copyTo(outImg3);
  
  gimage1.copyTo(outImg41);
  warpPerspective(gimage2, result4, H, result3.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  
  imshow("result", result);
  imshow("result2", result2);
  
  imshow("result H", result3);
  imshow("result H2", result4);
  
  //imwrite("out.png", result);
  waitKey(0);
}

void featuresReceived(const mobots_msgs::FeatureSetWithDeltaPose& featuresMsg){
  cout << "received features" << endl;
  ImageFeatures* features;
  if(pos++ == 0){
    features = &features1;
  }else{
    features = &features2;
  }
  MessageBridge::copyToCvStruct(featuresMsg, *features); 
  if(pos == 2){
    cout << "meh" << endl;
    checkResult();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "testFeatures");
  if(argc < 3){
    cout << "not enough arguments" << endl;
    return 1;
  }
  gimage1 = imread(argv[1], 1); //1 for colours
  gimage2 = imread(argv[2], 1);
  Mat image1Gray;
  Mat image2Gray;
  cvtColor(gimage1, image1Gray, CV_RGB2GRAY); //FeatureDetecter etc. arbeiten alle auf Graustufenbildern
  cvtColor(gimage2, image2Gray, CV_RGB2GRAY);
  mobots_msgs::ImageWithPoseDebug i1;
  mobots_msgs::ImageWithPoseDebug i2;
  copyMatToImageMSg(image1Gray, i1);
  copyMatToImageMSg(image2Gray, i2);
  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriber = nodeHandle.subscribe("FeatureSetWithDeltaPose", 2, featuresReceived);
  ros::Publisher publisher = nodeHandle.advertise<mobots_msgs::ImageWithPoseDebug>("ImageWithPose", 2);
  cout << "now sending" << endl;
  sleep(1);
  publisher.publish(i1);
  publisher.publish(i2);
  cout << "both sent" << endl;
  ros::spin();
}

/*int main(int argc, char** argv){
  if(argc < 3){
    cout << "not enough arguments" << endl;
    return 1;
  }
  image1 = imread(argv[1], 1); //1 for colours
  image2 = imread(argv[2], 1);
  Mat image1Gray;
  Mat image2Gray;
  cvtColor(image1, image1Gray, CV_RGB2GRAY); //FeatureDetecter etc. arbeiten alle auf Graustufenbildern
  cvtColor(image2, image2Gray, CV_RGB2GRAY);
  double time = (double)getTickCount();
  Ptr<FeaturesFinder> finder = new SurfFeaturesFinder(400, 3, 4, 4, 2, false);
  //Ptr<FeaturesFinder> finder = new OrbFeaturesFinder(1000);
  Ptr<FeaturesMatcher> matcher = new CpuFeaturesMatcher(CpuFeaturesMatcher::SURF_DEFAULT);
  ImageFeatures features1;
  ImageFeatures features2;
  Delta delta;
  finder->findFeatures(image1Gray, features1);
  finder->findFeatures(image2Gray, features2);
  bool matchResult = matcher->match(features1, features2, delta);
  if(!matchResult){
    cout << "images do not overlap at all" << endl;
    return 1;
  }
  cout << "deltaX " << delta.x << endl;
  cout << "deltaY " << delta.y << endl;
  cout << "theta " << delta.theta << " rad = " << toDegree(delta.theta) << "°" << endl;
  Mat aff;
  findRotationMatrix2D(Point2f(0,0), delta.theta, aff);
  aff.at<double>(0,2) = delta.x;
  aff.at<double>(1,2) = delta.y;
  cout << "affen mat: " << endl << aff << endl;
  Mat result;
  Mat result2;
  result2.create(Size(image1.cols+image2.cols, image1.rows+image2.rows), image2.type());
  result.create(Size(image1.cols+image2.cols, image1.rows+image2.rows), image2.type());
  Mat outImg1 = result(Rect(0, 0, image1.cols, image1.rows));
  Mat outImg21 = result2(Rect(0, 0, image1.cols, image1.rows));

  warpAffine(image2, result, aff, result.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  image1.copyTo(outImg1);
  
  image1.copyTo(outImg21);
  warpAffine(image2, result2, aff, result.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  
  cout << "time in s: " << ((double)getTickCount() - time)/getTickFrequency() << endl;
  imshow("result", result);
  imshow("result2", result2);
  //imwrite("out.png", result);
  waitKey(0);
}*/
