#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "FeaturesFinder.h"
#include "FeaturesMatcher.h"

using namespace cv;
using namespace std;

Mat image1; //for debug
Mat image2;

inline double toDegree(double rad){
  return rad * 180 / 3.14159265;
}

/*
 * The first image will be stitched to the second one
 */
int main(int argc, char **argv){
  if(argc == 1){
    argv = (char**)calloc(128, sizeof(char)); //should be enough..
    argv[1] = "map2_30degrees.png";
    argv[2] = "map2.png";
  }
  image1 = imread(argv[1], 1); //1 for colours
  image2 = imread(argv[2], 1);
  Mat image1Gray;
  Mat image2Gray;
  cvtColor(image1, image1Gray, CV_RGB2GRAY); //FeatureDetecter etc. arbeiten alle auf Graustufenbildern
  cvtColor(image2, image2Gray, CV_RGB2GRAY);
  double time = (double)getTickCount();
  Ptr<FeaturesFinder> finder = new SurfFeaturesFinder();
  Ptr<FeaturesMatcher> matcher = new CpuFeaturesMatcher(CpuFeaturesMatcher::SURF_DEFAULT);
  ImageFeatures features1;
  ImageFeatures features2;
  ImageMatchResult matchResult;
  finder->findFeatures(image1Gray, features1);
  finder->findFeatures(image2Gray, features2);
  matcher->match(features1, features2, matchResult);
  cout << "deltaX " << matchResult.delta.x << endl;
  cout << "deltaY " << matchResult.delta.y << endl;
  cout << "theta " << matchResult.delta.theta << " rad = " << toDegree(matchResult.delta.theta) << "°" << endl;
  Mat result;
  result.create(Size(image1.cols+image2.cols, image1.rows+image2.rows), image2.type());
  Mat outImg2 = result(Rect(0, 0, image2.cols, image2.rows));
  image2.copyTo(outImg2);
  //without INTER_CUBIC the result looks like shit if there is a rotation, but it could be expensive
  warpPerspective(image1, result, matchResult.H, result.size(), INTER_CUBIC, BORDER_TRANSPARENT);
  cout << "time in s: " << ((double)getTickCount() - time)/getTickFrequency() << endl;
  imshow("result", result);
  imwrite("out.png", result);
  waitKey(0);
}