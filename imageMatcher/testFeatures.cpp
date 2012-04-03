#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "FeaturesFinder.h"
#include "FeaturesMatcher.h"

using namespace cv;
using namespace std;

Mat image1; //for debug
Mat image2;

int main(int argc, char **argv){
  image1 = imread(argv[1], 1);
  image2 = imread(argv[2], 1);
  Mat image1Gray;
  Mat image2Gray;
  cvtColor(image1, image1Gray, CV_RGB2GRAY);
  cvtColor(image2, image2Gray, CV_RGB2GRAY);
  Ptr<FeaturesFinder> finder = new SurfFeaturesFinder();
  Ptr<FeaturesMatcher> matcher = new CpuFeaturesMatcher(CpuFeaturesMatcher::SURF_DEFAULT);
  ImageFeatures features1;
  ImageFeatures features2;
  Delta delta;
  finder->findFeatures(image1Gray, features1);
  finder->findFeatures(image2Gray, features2);
  matcher->findDelta(features1, features2, delta);
  cout << "deltaX " << delta.x << endl;
  cout << "deltaY " << delta.y << endl;
  cout << "rot " << delta.theta << endl;
  int maxSize2 = sqrt(image2.cols*image2.cols + image2.rows*image2.rows);
  int maxWidth = max(image1.cols, maxSize2) + abs(delta.x);
  int maxHeight = max(image1.rows, maxSize2) + abs(delta.y);
  int centerX = maxWidth/2;
  int centerY = maxHeight/2;
  Mat result;
  result.create(Size(900, 900), CV_MAKETYPE(image1.depth(), 3));
  Mat outImg1 = result(Rect(0, 0, image1.cols, image1.rows));
  cout << maxWidth << "w " << maxHeight << "h" << endl;
  cout << centerX-image1.cols/2+delta.x << "x " << centerY-image1.rows/2+delta.y << " y" << endl;
  cout << maxSize2 << "width " << maxSize2 << "height" << endl;
  Mat outImg2 = result(Rect(delta.x, image1.rows, maxSize2, maxSize2));
  Point2f center(image2.cols/2, image2.rows/2);
  Mat rotMat = getRotationMatrix2D(center, delta.theta, 1.0);
  warpAffine(image2, outImg2, rotMat, outImg2.size(), INTER_LINEAR, BORDER_TRANSPARENT);
  image1.copyTo(outImg1);
  imshow("result", result);
  waitKey(0);
}