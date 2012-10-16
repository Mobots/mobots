#include "ImageHandler.h"
#include <pthread.h>
#include <iostream>
#include "geometry_msgs/Pose2D.h"
#include "feature_detector/MessageBridge.h"
#include "feature_detector/FeaturesFinder.h"

using namespace std;
using namespace cv;

//debug and shit
cv::Mat gimage1;
cv::Mat gimage2;
extern Mat H;

const char TAG[] = "[ImageHandler] ";


void copyMatToImageMSg(const cv::Mat& in, mobots_msgs::ImageWithPoseAndID& out2){
  sensor_msgs::Image* out = &out2.image;
  out->height = in.rows;
  out->width = in.cols;
  out->encoding = "mono8";
  out->is_bigendian = 0;
  out->step = in.cols * in.elemSize();
  out->data.resize(in.rows * out->step);
  if(in.isContinuous()){
    cout << "copyMat is continous" << endl;
    memcpy(&out->data[0], in.data, in.rows * out->step);
  }else{
    cout << "copyMat is not continous" << endl;
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
  featureSetSubscriber = nh.subscribe("FeatureSetWithDeltaPose", 2, &ImageHandler::featuresCallback, this);
}

void ImageHandler::shutterCallback(const mobots_msgs::ImageWithPoseAndID &imageWithPoseAndId){
  cout << TAG << "shutter" << endl;
  if(shutterPos == 0){
    cv_bridge::CvImagePtr a1 =  cv_bridge::toCvCopy(imageWithPoseAndId.image);
    gimage1 = a1->image;
  }else{
    cv_bridge::CvImagePtr b1 = cv_bridge::toCvCopy(imageWithPoseAndId.image);
    gimage2 = b1->image;
  }
  shutterPos++;
  shutterPos %= 2;
}

void ImageHandler::featuresCallback(const mobots_msgs::FeatureSetWithPoseAndID &featuresMsg){
  if(featurePos == 1){
    MessageBridge::copyToCvStruct(featuresMsg.features, features2);
    featurePos = 0;
      //just ugly copy from testFeatures.cpp
    geometry_msgs::Pose2D delta;
    Ptr<FeaturesMatcher> matcher = new CpuFeaturesMatcher("BruteForce-Hamming");
    bool matchResult = matcher->match(features1, features2, delta);
    if(matchResult){
      //cout << "images do not overlap at all" << endl;
      //return;
    //}
    
    time_t time(0);
    stringstream ss;
    ss << "img1" << time;
    imwrite(ss.str(), gimage1);
    ss.str("");
    ss.clear();
    ss << "img2" << time;
    imwrite(ss.str(), gimage2);
    
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
    }
    
    /*Mat aff2;
    findRotationMatrix2D(Point2f(0,0), delta2.theta, aff2);
    aff2.at<double>(0,2) = delta2.x;
    aff2.at<double>(1,2) = delta2.y;
    cout << "affen mat2: " << endl << aff2 << endl;
    
    Mat result3;
    Mat result4;
    result3.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
    result4.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
    Mat outImg3 = result3(Rect(0, 0, gimage1.cols, gimage1.rows));
    Mat outImg41 = result4(Rect(0, 0, gimage1.cols, gimage1.rows));

    warpAffine(gimage2, result3, aff2, result3.size(), INTER_CUBIC, BORDER_TRANSPARENT);
    gimage1.copyTo(outImg3);

    gimage1.copyTo(outImg41);
    warpAffine(gimage2, result4, aff2, result3.size(), INTER_CUBIC, BORDER_TRANSPARENT);

    imshow("result new method", result3);
    imshow("result new method 2", result4);
    
    /*cout << "H mat " << endl << H << endl;
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
    imshow("result H2", result4);*/
    waitKey(0);
  }else{
    MessageBridge::copyToCvStruct(featuresMsg.features, features1);
    featurePos++;
  }
}