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

void findRotationMatrix2D(Point2d center, double angle, Mat& rotMat){
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
extern Mat kpoints1;
extern Mat kpoints2;
extern Mat mm1;
extern Mat mm2;
extern Mat good_matches_r;
extern Mat good_matches;
extern Mat mega;
extern Mat homo;
int int1;
int int2;

void checkResult(){
  MatchResult result;
  Ptr<FeaturesMatcher> matcher = FeaturesMatcher::getDefault();
  bool matchResult = matcher->match(features1, features2, result);
  if(!matchResult){
    exit(1);
  }
	Mat aff;
  
  findRotationMatrix2D(Point2d(gimage2.cols/2, gimage2.rows/2), result.delta.theta, aff);
	Point2f center(gimage2.cols/2, gimage2.rows/2);
	Mat rotMatrix = getRotationMatrix2D(center, -result.delta.theta*180/M_PI, 1);
	Mat g2;
	warpAffine(gimage2, g2, rotMatrix, gimage2.size(), INTER_CUBIC, BORDER_TRANSPARENT);
	Point2f p1[3];
	p1[0] = Point2f(-1, 0);
	p1[1] = Point2f(0, 1);
	p1[2] = Point2f(1, 0);
	Point2f p2[3];
	for(int i = 0; i < 3; i++){
		p2[i].x = p1[i].x + result.delta.x;// - gimage2.rows/2;
		p2[i].y = p1[i].y + result.delta.y;// + gimage2.cols/2;
	}
	Mat shift = getAffineTransform(p1, p2);
	/*Mat shiftMatrix;
	shiftMatrix.create(2, 3, CV_64F);
	shiftMatrix.at<double>(0,0) = 1;
	shiftMatrix.at<double>(0,1) = 0;
	shiftMatrix.at<double>(0,2) = -result.delta.x + gimage1.cols/2;
	shiftMatrix.at<double>(1,0) = 0;
	shiftMatrix.at<double>(1,1) = 1;
	shiftMatrix.at<double>(1,2) = -result.delta.y + gimage1.rows/2;*/
	
	aff.at<double>(0,2) = result.delta.x;
  aff.at<double>(1,2) = result.delta.y;
  cout << "aff:" << endl << aff << endl;
	cout << "varX: " << result.varX << " varY: " << result.varY << " varTheta: " << result.varTheta << endl;
	
  Mat result1;
  Mat result2;
  result2.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  result1.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  Mat outImg1 = result1(Rect(0, 0, gimage1.cols, gimage1.rows));
  Mat outImg21 = result2(Rect(0, 0, gimage1.cols, gimage1.rows));

  warpAffine(g2, result1, shift, result1.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  gimage1.copyTo(outImg1);
  
  gimage1.copyTo(outImg21);
  warpAffine(g2, result2, shift, result1.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  
	Mat a = result1.clone();
	Mat b = result2.clone();

	stringstream ss1,ss2;
	ss1 << int1 << "-" << int2 << ".png";
	ss2 << int2 << "-" << int1 << ".png";
	imwrite(ss1.str(), a);
	imwrite(ss2.str(), b);
	exit(0);
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
  ros::init(argc, argv, "makeMatch", ros::init_options::AnonymousName);
  if(argc < 5){
    cout << "not enough arguments" << endl;
    return 1;
  }
  int1 = atoi(argv[3]);
	int2 = atoi(argv[4]);
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
  ros::Publisher publisher = nodeHandle.advertise<mobots_msgs::ImageWithPoseAndID>("image_pose_id_relay", 2);
  cout << "now sending" << endl;
  sleep(1);
  publisher.publish(i1);
  publisher.publish(i2);
  cout << "both sent" << endl;
  ros::spin();
}