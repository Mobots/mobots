#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "FeaturesMatcher.h"
#include "FeaturesFinder.h"
#include "combination.hpp"

using namespace std;
using namespace cv;

extern Mat image1; //DEBUG
extern Mat image2;
extern Mat aff;

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

bool isSimilar(Mat& affine1, Mat& affine2){
  return 
  abs(affine1.at<double>(0,0) - affine1.at<double>(0,0)) < 2 &&
  abs(affine1.at<double>(0,1) - affine1.at<double>(0,1)) < 2 &&
  abs(affine1.at<double>(0,2) - affine1.at<double>(0,2)) < 2 &&
  abs(affine1.at<double>(1,0) - affine1.at<double>(1,0)) < 2 &&
  abs(affine1.at<double>(1,1) - affine1.at<double>(1,1)) < 2 &&
  abs(affine1.at<double>(1,2) - affine1.at<double>(1,2)) < 2;
}

Mat* findAffine(vector<Point2f>& points1, vector<Point2f>& points2){
  vector<Mat> mats;
  vector<int> indices(points1.size());
  int size = points1.size();
  for(int i = 0; i < size; i++)
    indices[i] = i;
  
  while (boost::next_combination(indices.begin(), indices.begin() + 3, indices.end())){
    Mat affine = getAffineTransform(&points1[indices[0]], &points2[indices[0]]);
    printf("indices %i, %i, %i", indices[0], indices[1], indices[2]);
    for(int i = 0; i < mats.size(); i++){
      if(isSimilar(mats[i], affine)){
	cout << "found similar" << endl;
	cout << "affine1: " << endl << affine << endl;
	cout << "affine2: " << endl << mats[i] << endl;
	aff = affine;
	return 0;	
      }
    }
    mats.push_back(affine);
    //cout << affine << endl;
  }
  return 0;
}

void symmetryTest(const vector<vector<DMatch> >& matches1, const vector<vector<DMatch> >& matches2, 
		  vector<DMatch>& symMatches){
  for(vector<vector<DMatch> >::const_iterator matchIterator1 = matches1.begin(); 
      matchIterator1 != matches1.end(); matchIterator1++){
    if(matchIterator1->size() < 2)
      continue;
    for(vector<vector<DMatch> >::const_iterator matchIterator2 = matches2.begin();
	matchIterator2 != matches2.end(); matchIterator2++){
      if(matchIterator2->size() < 2)
	continue;
      if((*matchIterator1)[0].queryIdx ==
	(*matchIterator2)[0].trainIdx &&
	(*matchIterator2)[0].queryIdx ==
	(*matchIterator1)[0].trainIdx){
	symMatches.push_back(
		      DMatch((*matchIterator1)[0].queryIdx,
			    (*matchIterator1)[0].trainIdx,
			    (*matchIterator1)[0].distance)
			    );
	break;
      }
    }
  }
}

float ratio = 0.65;

void ratioTest(vector<vector<DMatch> >& matches){
  for(vector<vector<DMatch> >::iterator matchIterator = matches.begin(); matchIterator != matches.end(); matchIterator++){
    if(matchIterator->size() < 2)
      matchIterator->clear();
    else{
      if((*matchIterator)[0].distance / (*matchIterator)[1].distance > ratio)
	matchIterator->clear();
    }
  }
}

/*
 * from opencv cookbook
 */
void match2(const DescriptorMatcher* matcher, ImageFeatures img1, const ImageFeatures img2, ImageMatchResult result){
  vector<vector<DMatch> > matches1;
  vector<vector<DMatch> > matches2;
  matcher->knnMatch(img1.descriptors, img2.descriptors, matches1, 2);
  matcher->knnMatch(img2.descriptors, img1.descriptors, matches2, 2);
  cout << "matches: " << matches1.size() << endl;
  ratioTest(matches1);
  ratioTest(matches2);
  vector<DMatch> good_matches;
  symmetryTest(matches1, matches2, good_matches);
  cout << "symmetric matches: " << good_matches.size() << endl;
  vector<Point2f> points1;
  vector<Point2f> points2;
  for(int i = 0; i < good_matches.size(); i++){
    points1.push_back(img1.keypoints[good_matches[i].queryIdx].pt);
    points2.push_back(img2.keypoints[good_matches[i].trainIdx].pt);
  }
  Mat H = findHomography(points1, points2, RANSAC);
  aff = H;
  std::cout << "method 2 H: " << endl;
  cout << toDegree(acos(H.at<double>(0,0))) << "° " << toDegree(asin(-H.at<double>(0,1))) << "° "
  << H.at<double>(0,2) << endl;
  cout << toDegree(asin(H.at<double>(1,0)))  << "° " << toDegree(acos(H.at<double>(1,1))) << "° " 
  << H.at<double>(1,2) << endl;
  cout << H.at<double>(2,0) << "  " << H.at<double>(2,1) << "  " << H.at<double>(2,2) << endl;
  
    Mat img_matches;
  drawMatches(image1, img1.keypoints, image2, img2.keypoints,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  imshow("method2 good Matches", img_matches);
}

void CpuFeaturesMatcher::match(const ImageFeatures& img1, const ImageFeatures& img2, ImageMatchResult& result) const{
  std::vector<DMatch> matches;
  std::vector<DMatch> good_matches;
    
  matcher->match(img1.descriptors, img2.descriptors, matches);
  /*Ptr<FlannBasedMatcher> matcher2 = new FlannBasedMatcher();
  vector<vector<DMatch> > matches2;
  matcher2->radiusMatch(img1.descriptors, img2.descriptors, matches2, 0.0001);
  Mat out;
  drawMatches(image1, img1.keypoints, image2, img2.keypoints,
               good_matches, out, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), 0);
  imshow( "Good Matches", out);
  waitKey(0);*/
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
  cout << img1.keypoints.size() << " features in img1, " << img2.keypoints.size() << " features in img2" << endl;
  cout << "min distance: " << min_dist << endl;
  cout << "match count: " << matches.size() << endl;
  cout << "good match count: " << good_matches.size() << endl;  
  std::vector<Point2f> points1;
  std::vector<Point2f> points2;
  for(int i = 0; i < good_matches.size(); i++){
    points1.push_back(img1.keypoints[good_matches[i].queryIdx].pt);
    points2.push_back(img2.keypoints[good_matches[i].trainIdx].pt);
  }
  Mat transformMatrix = findHomography(points1, points2, RANSAC);
  //findAffine(points1, points2);
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
  
  std::cout << "H-Mat" << endl;
  cout << toDegree(acos(transformMatrix.at<double>(0,0))) << "° " << toDegree(asin(-transformMatrix.at<double>(0,1))) << "° "
  << transformMatrix.at<double>(0,2) << endl;
  cout << toDegree(asin(transformMatrix.at<double>(1,0)))  << "° " << toDegree(acos(transformMatrix.at<double>(1,1))) << "° " 
  << transformMatrix.at<double>(1,2) << endl;
  cout << transformMatrix.at<double>(2,0) << "  " << transformMatrix.at<double>(2,1) << "  " << transformMatrix.at<double>(2,2) << endl;
  
    //-- DEBUG Show detected matches
  Mat img_matches;
  drawMatches(image1, img1.keypoints, image2, img2.keypoints,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  imshow("Good Matches", img_matches);
  
  match2(&(*matcher), img1, img2, result);
}



