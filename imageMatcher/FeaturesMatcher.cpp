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

double toDegree(double rad){
  return rad * 180 / 3.14159265;
}

void CpuFeaturesMatcher::findDelta(const ImageFeatures& img1, const ImageFeatures& img2, Delta& delta) const{
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
    //if(good_matches.size() > 2) break;
  }
  cout << "match count " << good_matches.size() << endl;  
  std::vector<Point2f> points1;
  std::vector<Point2f> points2;
  for(int i = 0; i < good_matches.size(); i++){
    points1.push_back(img1.keypoints[good_matches[i].queryIdx].pt);
    points2.push_back(img2.keypoints[good_matches[i].trainIdx].pt);
  }
  //Mat affine = getAffineTransform(points1, points2);
  //cout << "affine " << affine << endl;
  Mat transformMatrix = findHomography(points1, points2, CV_RANSAC, 3);
  delta.x = transformMatrix.at<double>(0, 2);
  delta.y = transformMatrix.at<double>(1, 2);
  delta.theta = min(toDegree(sin(transformMatrix.at<double>(0,1))), toDegree(cos(transformMatrix.at<double>(0,0))));
  
  std::cout << "Mat " << transformMatrix << std::endl;
  cout << toDegree(cos(transformMatrix.at<double>(0,0)))  << "° " << toDegree(sin(transformMatrix.at<double>(0,1))) << "°" << endl;
  cout << toDegree(-sin(transformMatrix.at<double>(1,0))) << "° " << toDegree(cos(transformMatrix.at<double>(1,1))) << "°" << endl;
  
    //-- DEBUG Show detected matches
  Mat img_matches;
  drawMatches(image1, img1.keypoints, image2, img2.keypoints,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), 0);
  imshow( "Good Matches", img_matches );
}



