#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>

#include "draw.h"
#include "ror3.h"
#include "ror.h"
#include "profile.h"
#include "feature_detector/FeaturesMatcher.h"

using namespace std;
using namespace cv;

extern Mat gimage1; //DEBUG
extern Mat gimage2;
Mat H;

const char CpuFeaturesMatcher::SURF_DEFAULT[] = "FlannBased";
const char CpuFeaturesMatcher::ORB_DEFAULT[] = "BruteForce-Hamming";
static const int LENGTHDIFF_THRESHOLD = 2;    //if abs(distance(a) - distance(b)) > => outlier

CpuFeaturesMatcher::CpuFeaturesMatcher(const string& type){
  matcher = DescriptorMatcher::create(type);
}
Ptr<FeaturesMatcher> FeaturesMatcher::getDefault(){
  return new CpuFeaturesMatcher(CpuFeaturesMatcher::ORB_DEFAULT);
}

inline double toDegree(double rad){
  return rad * 180 / 3.14159265;
}

/*
 * from opencv cookbook
 */
static void symmetryTest(const vector<vector<DMatch> >& matches1, const vector<vector<DMatch> >& matches2, 
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

static inline double euclideanDistance(const Point2f& p1, const Point2f& p2){
  double yDiff = double(p2.y)-double(p1.y);
  double xDiff = double(p2.x)-double(p1.x);
  return sqrt(xDiff*xDiff + yDiff*yDiff);
}

static void planeTest(const vector<Point2f>& points1, const vector<Point2f>& points2, vector<Point2f>& out1, vector<Point2f>& out2){
  const int p1Size = points1.size();
  int min = 4;
  for(int i1 = 0; i1 < p1Size; i1++){
    int curr = 0;
    for(int i2 = 0; i2 < p1Size; i2++){
      Point2f p11 = points1[i1];
      Point2f p12 = points1[i2];
      Point2f p21 = points2[i1];
      Point2f p22 = points2[i2];
      if(abs(
	euclideanDistance(p11, p12) 
	- euclideanDistance(p21, p22)
	    ) > LENGTHDIFF_THRESHOLD){
	  //cout << "sorted out " << i1 << "-" << i2 << endl;
	continue;
      }
      curr++;
      if(curr >= min){
	out1.push_back(p11);
	out2.push_back(p21);
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

/*Delta delta2;
Mat affine3;*/
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
  vector<Point2f> points1Refined;
  vector<Point2f> points2Refined;
  planeTest(points1, points2, points1Refined, points2Refined);
  /*Mat m = estimateRigidTransform(points2Refined, a, false);
  cout << "mega " << endl << m << endl;
  double d1 = atan2(-m.at<double>(0,1), m.at<double>(0,0));
  double d2 = acos(m.at<double>(0,0));
  cout << "d1 " << d1 << " = " << toDegree(d1) << "° d2 " << d2 << " = " << toDegree(d2) << "°" << endl;*/
  vector<uchar> status;
  findHomography(points2Refined, points1Refined, CV_RANSAC, 3, status);
  vector<Point2f> points1Refinedx2;
  vector<Point2f> points2Refinedx2;
  for(int i = 0; i < status.size(); i++){
    if(status[i]){
      points1Refinedx2.push_back(points1Refined[i]);
      points2Refinedx2.push_back(points2Refined[i]);
    }
  }
  Mat d = estimateRigidTransform(points2Refinedx2, points1Refinedx2, false);
  /*cout << "new d " << endl << d << endl;
  affine3 = d;
  d = estimateRigidTransform(points2Refined, a, true);
  cout << "new d with true" << endl << d << endl;*/
  delta.theta = atan2(-d.at<double>(0,1), d.at<double>(0,0));
  delta.x = d.at<double>(0,2);
  delta.y = d.at<double>(1,2);
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