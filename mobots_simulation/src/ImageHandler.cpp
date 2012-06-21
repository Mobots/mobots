#include "ImageHandler.h"

using namespace std;
using namespace cv;

//debug and shit
cv::Mat gimage1;
cv::Mat gimage2;

ImageHandler::ImageHandler():
  imagePos(0), shutterPos(0), featurePos(0){
  ros::NodeHandle nh;
  publisher = nh.advertise<mobots_msgs::ImageWithPoseDebug>("ImageWithPose", 2);
  featureSetSubscriber = nh.subscribe("FeatureSetWithDeltaPose", 2, &ImageHandler::featuresCallback, this);
  imageSubscriber = nh.subscribe("/usb_cam/image_raw", 2, &ImageHandler::imageCallback, this);
}

void ImageHandler::shutterCallback(){
  std::cout << "shutter" << std::endl;
  if(shutterPos == 0){
    gimage1 = cv_bridge::toCvCopy(images[0])->image;
  }else{
    gimage2 = cv_bridge::toCvCopy(images[1])->image;
  }
  mobots_msgs::ImageWithPoseDebug msg;
  msg.image = images[shutterPos];
  publisher.publish(msg);
  shutterPos++;
  shutterPos %= 2;
}

void ImageHandler::shutterCallback2(const mobots_msgs::ImagePoseID imageWithPoseAndId){
  imageCallback(imageWithPoseAndId.image);
  shutterCallback();
}

void ImageHandler::findRotationMatrix2D(Point2f center, double angle, Mat& rotMat){
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

inline double ImageHandler::toDegree(double rad){
  return rad * 180 / M_PI;
}

void ImageHandler::imageCallback(const sensor_msgs::Image image){
  images[shutterPos] = image;
  imagePos++;
  imagePos %= 2;
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