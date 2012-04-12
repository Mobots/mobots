#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "FeaturesMatcher.h"
#include "FeaturesFinder.h"

using namespace std;
using namespace cv;

extern Mat image1; //DEBUG
extern Mat image2;

const char CpuFeaturesMatcher::SURF_DEFAULT[] = "FlannBased";
const char CpuFeaturesMatcher::ORB_DEFAULT[] = "BruteForce-Hamming";

/**
 * matchers: BruteForce, Bruteforce-L1, BruteForce-Hamming, BruteForce-HammingLUT, FlannBased
 */
CpuFeaturesMatcher::CpuFeaturesMatcher(const string& type){
  matcher = DescriptorMatcher::create(type);
}

inline double avg(double d1, double d2){
  return (d1+d2)/2;
}

inline double toDegree(double rad){
  return rad * 180 / 3.14159265;
}

void CpuFeaturesMatcher::match(const ImageFeatures& img1, const ImageFeatures& img2, ImageMatchResult& result) const{
  std::vector<DMatch> matches;
  std::vector<DMatch> good_matches;
    
  matcher->match(img1.descriptors, img2.descriptors, matches);
  double min_dist = 100;
  int N = img1.descriptors.rows;
  for(int i = 0; i < N; i++){ 
    double dist = matches[i].distance;
    if(dist < min_dist && dist > 0) min_dist = dist;
  }
  for(int i = 0; i < N; i++){ 
    if(matches[i].distance < 2*min_dist)
     good_matches.push_back(matches[i]); 
  }
  cout << "match count " << good_matches.size() << endl;  
  std::vector<Point2f> points1;
  std::vector<Point2f> points2;
  for(int i = 0; i < good_matches.size(); i++){
    points1.push_back(img1.keypoints[good_matches[i].queryIdx].pt);
    points2.push_back(img2.keypoints[good_matches[i].trainIdx].pt);
  }
  Mat transformMatrix = findHomography(points1, points2, CV_RANSAC);
  /* Matrix form:
   * cos(theta)  -sin(theta) deltaX
   * sin(theta)   cos(theta) deltaY
   *    0             0        1
   */
  result.delta.x = transformMatrix.at<double>(0, 2);
  result.delta.y = transformMatrix.at<double>(1, 2);
  double majorDiag = acos(avg( transformMatrix.at<double>(0,0), transformMatrix.at<double>(1,1)));
  double minorDiag = asin(avg(-transformMatrix.at<double>(0,1), transformMatrix.at<double>(1,0)));
  result.delta.theta = avg(majorDiag, minorDiag);
  result.H = transformMatrix;
  
  std::cout << "rot Mat" << endl;
  cout << toDegree(acos(transformMatrix.at<double>(0,0)))  << "° " << toDegree(asin(-transformMatrix.at<double>(0,1))) << "°" << endl;
  cout << toDegree(asin(transformMatrix.at<double>(1,0))) << "° " << toDegree(acos(transformMatrix.at<double>(1,1))) << "°" << endl;
  
    //-- DEBUG Show detected matches
  Mat img_matches;
  drawMatches(image1, img1.keypoints, image2, img2.keypoints,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), 0);
  imshow( "Good Matches", img_matches );
}



