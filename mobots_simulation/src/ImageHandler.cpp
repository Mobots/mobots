#include "ImageHandler.h"
#include <pthread.h>

using namespace std;
using namespace cv;

//debug and shit

cv::Mat gimage1;
cv::Mat gimage2;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

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

inline double toDegree(double rad){
  return rad * 180 / M_PI;
}

ImageHandler::ImageHandler():
  shutterPos(0), featurePos(0){
  ros::NodeHandle nh;
  publisher = nh.advertise<mobots_msgs::ImageWithPoseDebug>("ImageWithPose", 2);
  featureSetSubscriber = nh.subscribe("FeatureSetWithDeltaPose", 2, &ImageHandler::featuresCallback, this);
  imageSubscriber = nh.subscribe("/usb_cam/image_raw", 2, &ImageHandler::imageCallback, this);
  /*Mat img1 = imread("/home/jonas/mobots/feature_detector/testImages/image001.png", 1);
  Mat img2 = imread("/home/jonas/mobots/feature_detector/testImages/image001.png", 1);
  gimage1 = img1;
  gimage2 = img2;
  mobots_msgs::ImageWithPoseDebug i1;
  mobots_msgs::ImageWithPoseDebug i2;
  copyMatToImageMSg(img1, i1);
  copyMatToImageMSg(img2, i2);
  cout << "now sending" << endl;
  sleep(1);
  publisher.publish(i1);
  publisher.publish(i2);
  cout << "both sent" << endl;*/
}

cv_bridge::CvImagePtr a1;
cv_bridge::CvImagePtr b1;

void ImageHandler::shutterCallback(){
  if(shutterPos == 1){
  Mat img1 = imread("/home/jonas/mobots/feature_detector/testImages/image001.png", 1);
  Mat img2 = imread("/home/jonas/mobots/feature_detector/testImages/image001.png", 1);
  gimage1 = img1;
  gimage2 = img2;
  mobots_msgs::ImageWithPoseDebug i1;
  mobots_msgs::ImageWithPoseDebug i2;
  copyMatToImageMSg(img1, i1);
  copyMatToImageMSg(img2, i2);
  cout << "now sending" << endl;
  sleep(1);
  //publisher.publish(i1);
  //publisher.publish(i2);
  cout << "both sent" << endl;
  }
  pthread_mutex_lock(&mutex);
  sensor_msgs::Image i;
  if(shutterPos == 0){
    i = image1;
    a1 =  cv_bridge::toCvCopy(i);
    gimage1 = a1->image;
  }
  else{
    i = image2;
    b1 = cv_bridge::toCvCopy(i);
    gimage2 = b1->image;
  }
  //sensor_msgs::Image i = shutterPos == 0 ? image1 : image2;
  std::cout << "shutter" << std::endl;
  /*if(shutterPos == 0){
    gimage1 = cv_bridge::toCvCopy(i)->image;
  }else{
    gimage2 = cv_bridge::toCvCopy(i)->image;
  }*/
  mobots_msgs::ImageWithPoseDebug msg;
  if(shutterPos == 0)
    copyMatToImageMSg(gimage1, msg);
  else
    copyMatToImageMSg(gimage2, msg);
  publisher.publish(msg);
  shutterPos++;
  shutterPos %= 2;
  pthread_mutex_unlock(&mutex);
}

void ImageHandler::shutterCallback2(const mobots_msgs::ImagePoseID imageWithPoseAndId){
  imageCallback(imageWithPoseAndId.image);
  shutterCallback();
}

void ImageHandler::imageCallback(const sensor_msgs::Image image){
  pthread_mutex_lock(&mutex);
  if(shutterPos == 0)
    image1 = image;
  else
    image2 = image;
  pthread_mutex_unlock(&mutex);
}

void ImageHandler::featuresCallback(const mobots_msgs::FeatureSetWithDeltaPose featuresMsg){
  if(featurePos == 1){
    MessageBridge::copyToCvStruct(featuresMsg, features2);
    featurePos = 0;
      //just ugly copy from testFeatures.cpp
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
    findRotationMatrix2D(Point2f(0,0), delta.theta, aff);
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
    waitKey(0);
  }else{
    MessageBridge::copyToCvStruct(featuresMsg, features1);
    featurePos++;
  }
}