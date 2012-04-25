#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "FeaturesMatcher.h"
#include "FeaturesFinder.h"
#include "ror.h"
#include "profile.h"

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

void printHMat(const Mat& H, const string title = string("")){
  cout << "H-Mat: " << title << endl;
  cout << toDegree(acos(H.at<double>(0,0))) << "° " << toDegree(asin(-H.at<double>(0,1))) << "° "
  << H.at<double>(0,2) << endl;
  cout << toDegree(asin(H.at<double>(1,0)))  << "° " << toDegree(acos(H.at<double>(1,1))) << "° " 
  << H.at<double>(1,2) << endl;
  cout << H.at<double>(2,0) << "  " << H.at<double>(2,1) << "  " << H.at<double>(2,2) << endl;
}

void printHMatRaw(const Mat& H, const string title = string("")){
  cout << "H-Mat: " << title << endl;
  cout << H << endl;
}

/*
 * from opencv cookbook
 */
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

/*
 * from opencv cookbook
 */
void ratioTest(vector<vector<DMatch> >& matches, float ratioThreshold){
  int removed = 0;
  for(vector<vector<DMatch> >::iterator matchIterator = matches.begin(); matchIterator != matches.end(); matchIterator++){
    if(matchIterator->size() < 2)
      matchIterator->clear();
    else{
      if((*matchIterator)[0].distance / (*matchIterator)[1].distance > ratioThreshold){
	matchIterator->clear();
	removed++;
      }
    }
  }
  cout << "removed: " << removed << endl;
}


/**
 * does the following:
 * if(value > 1)
 * 	value = 1;
 * else if(value < -1)
 * 	value = -1;
 */
void normalize(double& value){
  if(value > 1)
    value = 1;
  else if(value < -1)
    value = -1;
}

#define sgn(x) (( x > 0 ) - ( x < 0 ))

bool CpuFeaturesMatcher::match(const ImageFeatures& img1, const ImageFeatures& img2, Delta& delta) const{
  moduleStarted("cpu matcher");
  vector<vector<DMatch> > matches1;
  vector<vector<DMatch> > matches2;
  matcher->knnMatch(img1.descriptors, img2.descriptors, matches1, 2);
  matcher->knnMatch(img2.descriptors, img1.descriptors, matches2, 2);
  cout << "matches: " << matches1.size() << endl;
  ratioTest(matches1, ratioThreshold);
  ratioTest(matches2, ratioThreshold);
  vector<DMatch> good_matches;
  symmetryTest(matches1, matches2, good_matches);
  cout << "symmetric matches: " << good_matches.size() << endl;
  if(good_matches.size() < 5){
    return false;
  }
  vector<Point2f> points1;
  vector<Point2f> points2;
  for(int i = 0; i < good_matches.size(); i++){
    points1.push_back(img1.keypoints[good_matches[i].queryIdx].pt);
    points2.push_back(img2.keypoints[good_matches[i].trainIdx].pt);
  }
  moduleEnded();
  moduleStarted("get transform");
  bool ok = rorAlternative(points1, points2, delta);
  //Mat H = getAffineTransform(&points2[0], &points1[0]);
  //aff = H;
  //Mat H = findHomography(points2, points1, CV_RANSAC);
  //aff = H;
  /* Matrix form:
   * cos(theta)  -sin(theta) deltaX
   * sin(theta)   cos(theta) deltaY
   *    0             0        1
   */
  /*printHMat(H);
  printHMatRaw(H, "raw");
  double mainDiag = avg(  H.at<double>(0,0), H.at<double>(1,1));
  double minorDiag = avg(-H.at<double>(0,1), H.at<double>(1,0));
  normalize(mainDiag);
  normalize(minorDiag);
  mainDiag = acos(mainDiag);
  minorDiag = asin(minorDiag);
  //acos nimmt nur positive werte an
  mainDiag *= sgn(minorDiag);
  double theta = avg(mainDiag, minorDiag);
  delta.theta = theta;
  delta.x = H.at<double>(0,2);
  delta.y = H.at<double>(1,2);*/
  moduleEnded();
  Mat img_matches;
  drawMatches(image1, img1.keypoints, image2, img2.keypoints,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  imshow("method2 good Matches", img_matches);
  return ok;
}