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

void checkResult(){
  MatchResult result;
  Ptr<FeaturesMatcher> matcher = FeaturesMatcher::getDefault();
  bool matchResult = matcher->match(features1, features2, result);
  if(!matchResult){
    cout << "images do not overlap at all" << endl;
	 waitKey(0);
    return;
  }
	//imshow("good Matches with ransac", good_matches_r);
	//imshow("good Matches", good_matches);

  /*cout << "deltaX " << delta.x << endl;
  cout << "deltaY " << delta.y << endl;
  cout << "theta " << delta.theta << " rad = " << toDegree(delta.theta) << "°" << endl;
  Mat aff;*/
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
	//rotMatrix.at<double>(0,2) = result.delta.x;
  //rotMatrix.at<double>(1,2) = result.delta.y;
  cout << "aff:" << endl << aff << endl;
	cout << "varX: " << result.varX << " varY: " << result.varY << " varTheta: " << result.varTheta << endl;
	
	//imshow("mm1", mm1);
	//imshow("mm2", mm2);
	//imshow("mega", mega);
	
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
  imshow("result with ransac", a);
  imshow("result2 with ransac", b);
	
	/*Mat result3;
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
  
  imshow("result with homo + ransac", result3);
  imshow("result2 with homo + ransac", result4);*/
 
  
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
  ros::Subscriber subscriber = nodeHandle.subscribe("/mobot1/featureset_pose_id", 2, featuresReceived);
  ros::Publisher publisher = nodeHandle.advertise<mobots_msgs::ImageWithPoseAndID>("/mobot1/image_pose_id_relay", 2);
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
