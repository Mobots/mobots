#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>

#include "draw.h"
#include "ror3.h"
#include "ror.h"
#include "profile.h"

using namespace std;
using namespace cv;

extern Mat gimage1; //DEBUG
extern Mat gimage2;
Mat H;

const char CpuFeaturesMatcher::SURF_DEFAULT[] = "FlannBased";
const char CpuFeaturesMatcher::ORB_DEFAULT[] = "BruteForce-Hamming";

CpuFeaturesMatcher::CpuFeaturesMatcher(const string& type){
  matcher = DescriptorMatcher::create(type);
}
Ptr<FeaturesMatcher> FeaturesMatcher::getDefault(){
  return new CpuFeaturesMatcher(CpuFeaturesMatcher::ORB_DEFAULT);
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
    if(matchIterator1->size() == 0)
      continue;
    for(vector<vector<DMatch> >::const_iterator matchIterator2 = matches2.begin();
	matchIterator2 != matches2.end(); matchIterator2++){
      if(matchIterator2->size() == 0)
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

Delta delta2;
Mat affine3;
bool ror2(const vector<Point2f>& points1, const vector<Point2f>& points2, vector<Point2f>& out1, vector<Point2f>& out2);
bool CpuFeaturesMatcher::match(const FeatureSet& img1, const FeatureSet& img2, Delta& delta) const{
    vector<DMatch> matches;
  /*moduleStarted("only ror");
  matcher->match(img1.descriptors, img2.descriptors, matches);
  vector<Point2f> points11;
  vector<Point2f> points22;
  for(int i = 0; i < matches.size(); i++){
    points11.push_back(img1.keyPoints[matches[i].queryIdx].pt);
    points22.push_back(img2.keyPoints[matches[i].trainIdx].pt);
  }
  rorAlternative(points11, points22, delta2);
  moduleEnded();*/
  
  moduleStarted("cpu matcher + get transform");
  vector<vector<DMatch> > matches1;
  vector<vector<DMatch> > matches2;
  matcher->knnMatch(img1.descriptors, img2.descriptors, matches1, 2);
  matcher->knnMatch(img2.descriptors, img1.descriptors, matches2, 2);
  cout << "matches: " << matches1.size() << endl;
  ratioTest(matches1, ratioThreshold);
  ratioTest(matches2, ratioThreshold);
  vector<DMatch> good_matches;
  symmetryTest(matches1, matches2, good_matches);
  /*for(int i = 0; i < matches1.size(); i++){
    if(matches1[i].size() > 0)
      good_matches.push_back(matches1[i][0]);
  }*/
  cout << "symmetric matches: " << good_matches.size() << endl;
  /*if(good_matches.size() < 5){
    return false;
  }*/
  vector<Point2f> points1;
  vector<Point2f> points2;
  for(int i = 0; i < good_matches.size(); i++){
    points1.push_back(img1.keyPoints[good_matches[i].queryIdx].pt);
    points2.push_back(img2.keyPoints[good_matches[i].trainIdx].pt);
  }
  bool ok = rorAlternative(points1, points2, delta);
  moduleEnded();
  moduleStarted("new");
  vector<Point2f> a;
  vector<Point2f> b;
  ror2(points1, points2, a, b);
  Mat m = cv::estimateRigidTransform(b, a, false);
  cout << "mega " << endl << m << endl;
  double d1 = atan2(-m.at<double>(0,1), m.at<double>(0,0));
  double d2 = acos(m.at<double>(0,0));
  cout << "d1 " << d1 << " = " << toDegree(d1) << "° d2 " << d2 << " = " << toDegree(d2) << "°" << endl;
  vector<uchar> status;
  Mat c = findHomography(b, a, CV_RANSAC, 3, status);
  cout << "new c " << endl << c << endl;
  vector<Point2f> a2;
  vector<Point2f> b2;
  for(int i = 0; i < status.size(); i++){
    if(status[i]){
      a2.push_back(a[i]);
      b2.push_back(b[i]);
    }
  }
  cout << "a2/b2 size " << a2.size() << endl;
  Mat d = estimateRigidTransform(b, a, false);
  cout << "new d " << endl << d << endl;
  affine3 = d;
  d = estimateRigidTransform(b, a, true);
  cout << "new d with true" << endl << d << endl;
  moduleEnded();
  /*vector<Point2f> a;
  vector<Point2f> b;
  point* pa = new point[points1.size()];
  point* pb = new point[points1.size()];
  for(int i = 0; i < points1.size(); i++){
    pa[i].x = points1[i].x;
    pa[i].y = points1[i].y;
    pb[i].x = points2[i].x;
    pb[i].y = points2[i].y;
  }
  int* mask = new int[points1.size()];
  moduleStarted("real ror");
  ror(pa, pb, points1.size(), 20, mask);
  moduleEnded();
  int count = 0;
  vector<char> mask2;
  for(int i = 0; i < points1.size(); i++){
    if(mask[i]){
      a.push_back(points1[i]);
      b.push_back(points2[i]);
      mask2.push_back(1);
    }else
      mask2.push_back(0);
  }
  //ror2(points1, points2, a, b);
  
  /*  moduleStarted("cpu matcher, homo + ror");
  vector<uchar> mask;
  vector<Point2f> points11;
  vector<Point2f> points22;
  findHomography(points1, points2, CV_RANSAC, 3, mask);
  for(int i = 0; i < mask.size(); i++){
    if(mask[i]){
      points11.push_back(points1[i]);
      points22.push_back(points2[i]);
    }
  }
  cout << "size2: " << points22.size() << endl;
  rorAlternative(points11, points22, delta2);
  moduleEnded();*/
  /*if(a.size() > 2){
  H = getAffineTransform(&b[0], &a[0]);
  cout << "aff0" << endl << H << endl;
  //aff = H;*/
  //vector<uchar> inliers;
  //H = findHomography(points2, points1, CV_RANSAC);
  //cout << "H1" << endl << H << endl;
  //H = findHomography(b, a, CV_RANSAC, 1);
  //cout << "H2, real ror size from " << points1.size() << " to " << a.size() <<  endl << H << endl;
  //rorAlternative(a, b, delta2);
  //}
    
  /*vector<Point2f> v1;
  vector<Point2f> v2;
  cout << "inlier " << inliers.size() << endl;
  for(int i = 0; i < inliers.size(); i++){
    if(v1.size() == 3)
      break;
    if(!inliers[i]){
      v1.push_back(points1[i]);
      v2.push_back(points2[i]);
    }
  }
  cout << "v1 size " << v1.size() << endl;
  cout << "H" << endl << H << endl;
  if(v1.size() == 3){
  affine3 = getAffineTransform(v1, v2);
  //correctMatches() ?
  cout << "affine3" << endl << affine3 << endl;
  }
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
  /*Mat img_matches;
  drawing::drawMatches(gimage1, img1.keyPoints, gimage2, img2.keyPoints,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, 50);
  imshow("good Matches", img_matches);
  Mat matches3;
  drawing::drawMatches(gimage1, img1.keyPoints, gimage2, img2.keyPoints,
               good_matches, matches3, Scalar::all(-1), Scalar::all(-1),
               mask2, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, 50);
  imshow("real ror Matches", matches3);*/
  return ok;
}