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
ros::Subscriber subscriber;
ros::Publisher publisher;
int main2(int argc, char** argv);

bool done = false;
bool matchable = false;

void featuresReceived(const mobots_msgs::FeatureSetWithPoseAndID& featuresMsg);

int main(int argc, char** argv){
  ros::init(argc, argv, "schnauze");
  int picCount = 26;
  ros::NodeHandle nodeHandle;
  subscriber = nodeHandle.subscribe("/mobot1/featureset_pose_id", 2, featuresReceived);
  publisher = nodeHandle.advertise<mobots_msgs::ImageWithPoseAndID>("/mobot1/image_pose_id", 2);
	FILE* pf = fopen("/dev/null", "r"); //stdout stfu
  *stdout = *pf;
  string base = string("/home/jonas/mobots/slam") + string("/pics/karte2/");
  for(int i = 1; i < picCount; i++){
		for(int i2 = i+1; i2 < picCount; i2++){
		  stringstream ss1;
		  ss1 << base << i << ".png";
		  stringstream ss2;
		  ss2 << base << i2 << ".png";
		  char** paths = (char**) malloc(3*sizeof(char*));
		  char* path0 = (char*) calloc(1000, 1);
		  char* path1 = (char*) calloc(1000, 1);
		  char* path2 = (char*) calloc(1000, 1);
		  strcpy(path0, "schnauze");
		  strcpy(path1, ss1.str().c_str());
		  strcpy(path2, ss2.str().c_str());
		  paths[0] = path0;
		  paths[1] = path1;
		  paths[2] = path2;
		  main2(3, paths );
		  while(!done){
			 ros::spinOnce();
			 sleep(1);
		  }
		  done = false;
		  if(matchable)
			 cerr << i << ".png and " << i2 << ".png are matchable" << endl;
		  else if(abs(i-i2) <= 2)
			  cerr << i << ".png and " << i2 << ".png are NOT matchable" << endl;
		}
  }
}

void checkResult(){
  MatchResult mresult;
  Ptr<FeaturesMatcher> matcher = FeaturesMatcher::getDefault();
  bool matchResult = matcher->match(features1, features2, mresult);
  if(!matchResult){
  }
  matchable = matchResult;
  done = true;
  return;
	//imshow("good Matches with ransac", good_matches_r);
	//imshow("good Matches", good_matches);

  /*cout << "deltaX " << delta.x << endl;
  cout << "deltaY " << delta.y << endl;
  cout << "theta " << delta.theta << " rad = " << toDegree(delta.theta) << "°" << endl;
  Mat aff;*/
	Mat aff;
  
  findRotationMatrix2D(Point2d(gimage2.cols/2, gimage2.rows/2), -mresult.delta.theta, aff);
  aff.at<double>(0,2) = mresult.delta.x;
  aff.at<double>(1,2) = mresult.delta.y;
  cout << "aff:" << endl << aff << endl;
	
	//imshow("mm1", mm1);
	//imshow("mm2", mm2);
	imshow("mega", mega);
	
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
  
	Mat a = result.clone();
	Mat b = result2.clone();
  imshow("result with ransac", a);
  imshow("result2 with ransac", b);
	
	Mat result3;
  Mat result4;
  result3.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  result4.create(Size(gimage1.cols+gimage2.cols, gimage1.rows+gimage2.rows), gimage2.type());
  outImg1 = result3(Rect(0, 0, gimage1.cols, gimage1.rows));
  outImg21 = result4(Rect(0, 0, gimage1.cols, gimage1.rows));

	cout << "homo:" << endl << homo << endl;
  warpPerspective(gimage2, result3, homo, result.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  gimage1.copyTo(outImg1);
  
  gimage1.copyTo(outImg21);
  warpPerspective(gimage2, result4, homo, result.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  
  //imshow("result with homo + ransac", result3);
  //imshow("result2 with homo + ransac", result4);
 
  
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
	 pos = 0;
    checkResult();
  }
}

int main2(int argc, char** argv){
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
  cout << "now sending" << endl;
  sleep(1);
  publisher.publish(i1);
  publisher.publish(i2);
  cout << "both sent" << endl;
  ros::spinOnce();
}