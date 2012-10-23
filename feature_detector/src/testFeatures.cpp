#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "feature_detector/FeaturesFinder.h"
#include "feature_detector/MessageBridge.h"
#include "feature_detector/FeaturesMatcher.h"
#include "mobots_msgs/ImageWithPoseAndID.h"
#include "mobots_msgs/FeatureSetWithPoseAndID.h"



using namespace cv;
using namespace std;

Mat gimage1; //for debug
Mat gimage2;
extern Mat H;

char pos = 0;
FeatureSet features1;
FeatureSet features2;

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

void copyMatToImageMSg(const cv::Mat& in, mobots_msgs::ImageWithPoseAndID& out2){
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

extern Mat affine3;
extern Mat affine2;

void checkResult(){
  geometry_msgs::Pose2D delta;
  Ptr<FeaturesMatcher> matcher = FeaturesMatcher::getDefault();
  bool matchResult = matcher->match(features1, features2, delta, gimage1, gimage2);
  if(!matchResult){
    cout << "images do not overlap at all" << endl;
	 waitKey(0);
    return;
  }
  /*cout << "deltaX " << delta.x << endl;
  cout << "deltaY " << delta.y << endl;
  cout << "theta " << delta.theta << " rad = " << toDegree(delta.theta) << "°" << endl;
  Mat aff;
  
  findRotationMatrix2D(Point2f(gimage2.cols/2, gimage2.rows/2), delta.theta, aff);
  aff.at<double>(0,2) = delta.x;
  aff.at<double>(1,2) = delta.y;*/
  cout << affine3 << endl;
  Mat result;
  Mat result2;
  result2.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  result.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  Mat outImg1 = result(Rect(0, 0, gimage1.cols, gimage1.rows));
  Mat outImg21 = result2(Rect(0, 0, gimage1.cols, gimage1.rows));

  warpAffine(gimage2, result, affine3, result.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  gimage1.copyTo(outImg1);
  
  gimage1.copyTo(outImg21);
  warpAffine(gimage2, result2, affine3, result.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  
  imshow("result", result);
  imshow("result2", result2);
  
  cout << "now affine 2 " << endl << affine2 << endl;
  Mat result3;
  Mat result4;
  result3.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  result4.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  Mat outImg3 = result3(Rect(0, 0, gimage1.cols, gimage1.rows));
  Mat outImg41 = result4(Rect(0, 0, gimage1.cols, gimage1.rows));
  
  /*findRotationMatrix2D(Point2f(gimage2.cols/2, gimage2.rows/2), delta2.theta, aff);
  aff.at<double>(0,2) = delta2.x;
  aff.at<double>(1,2) = delta2.y;*/

  warpAffine(gimage2, result3, affine2, result3.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  gimage1.copyTo(outImg3);
  
  gimage1.copyTo(outImg41);
  warpAffine(gimage2, result4, affine2, result3.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  
  
  imshow("result with ransac", result3);
  imshow("result 2 with ransac", result4);
  
  /*Mat result3;
  Mat result4;
  result3.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  result4.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  Mat outImg3 = result3(Rect(0, 0, gimage1.cols, gimage1.rows));
  Mat outImg41 = result4(Rect(0, 0, gimage1.cols, gimage1.rows));

  warpPerspective(gimage2, result3, H, result3.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  gimage1.copyTo(outImg3);
  
  gimage1.copyTo(outImg41);
  warpPerspective(gimage2, result4, H, result3.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  
  
  imshow("result H", result3);
  imshow("result H2", result4);*/
  
  //imwrite("out.png", result);
  waitKey(0);
}

void featuresReceived(const mobots_msgs::FeatureSetWithPoseAndID& featuresMsg){
  cout << "received features" << endl;
  FeatureSet* features;
  if(pos++ == 0){
    features = &features1;
  }else{
    features = &features2;
  }
  MessageBridge::copyToCvStruct(featuresMsg.features, *features); 
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
  mobots_msgs::ImageWithPoseAndID i1;
  mobots_msgs::ImageWithPoseAndID i2;
  copyMatToImageMSg(image1Gray, i1);
  copyMatToImageMSg(image2Gray, i2);
  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriber = nodeHandle.subscribe("featureset_pose_id", 2, featuresReceived);
  ros::Publisher publisher = nodeHandle.advertise<mobots_msgs::ImageWithPoseAndID>("image_pose_id", 2);
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
