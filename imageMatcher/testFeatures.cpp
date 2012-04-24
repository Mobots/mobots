#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "FeaturesFinder.h"
#include "FeaturesMatcher.h"



using namespace cv;
using namespace std;

Mat image1; //for debug
Mat image2;
Mat aff;

inline double toDegree(double rad){
  return rad * 180 / 3.14159265;
}

int main(int argc, char** argv){
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
  Mat result;
  result.create(Size(image1.cols+image2.cols, image1.rows+image2.rows), image2.type());
  Mat outImg1 = result(Rect(0, 0, image1.cols, image1.rows));
  image1.copyTo(outImg1);
  /*Mat aff = getRotationMatrix2D(Point2f(0,0), -99.9, 1.0);
  aff.at<double>(0,2) = delta.x;
  aff.at<double>(1,2) = delta.y;*/
  warpAffine(image2, result, aff, result.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  cout << "time in s: " << ((double)getTickCount() - time)/getTickFrequency() << endl;
  imshow("result", result);
  //imwrite("out.png", result);
  waitKey(0);
}