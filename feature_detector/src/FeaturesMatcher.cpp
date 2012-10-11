#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>



#include "draw.h"
#include "profile.h"
#include "mobots_msgs/FeatureSet.h"
#include "feature_detector/FeaturesMatcher.h"

using namespace std;
using namespace cv;

extern Mat gimage1; //DEBUG
extern Mat gimage2;
Mat H;

const char CpuFeaturesMatcher::SURF_DEFAULT[] = "FlannBased";
const char CpuFeaturesMatcher::ORB_DEFAULT[] = "BruteForce-Hamming";
static const double LENGTHDIFF_THRESHOLD = 1.2;    //if abs(distance(a) - distance(b)) > => outlier
static const double MAX_ALLOWED_MATCH_DISTANCE = 60;
static const char* TAG = "[FeaturesMatcher] ";

CpuFeaturesMatcher::CpuFeaturesMatcher(const string& type){
  matcher = DescriptorMatcher::create(type);
}
Ptr<FeaturesMatcher> FeaturesMatcher::getDefault(){
  return new CpuFeaturesMatcher(CpuFeaturesMatcher::ORB_DEFAULT);
}

inline double toDegree(double rad){
  return rad * 180 / 3.14159265;
}

static inline double euclideanDistance(const Point2f& p1, const Point2f& p2){
  double yDiff = double(p2.y)-double(p1.y);
  double xDiff = double(p2.x)-double(p1.x);
  return sqrt(xDiff*xDiff + yDiff*yDiff);
}

/**
 * derived from opencv cookbook
 */
static void symmetryTest(const vector<DMatch>& matches1, const vector<DMatch>& matches2, 
		  const vector<KeyPoint>& kpoints1, const vector<KeyPoint>& kpoints2,
		  vector<Point2f>& points1, vector<Point2f>& points2){
  for(int i1 = 0; i1 < matches1.size(); i1++){
    for(int i2 = 0; i2 < matches2.size(); i2++){
		if(matches1[i1].queryIdx ==
		  matches2[i2].trainIdx &&
		  matches2[i2].queryIdx ==
		  matches1[i1].trainIdx &&
		  matches1[i1].distance < MAX_ALLOWED_MATCH_DISTANCE){
			 //cout << matches1[i1].distance << " | " <<  matches2[i2].distance << endl;
			 points1.push_back(kpoints1[matches1[i1].queryIdx].pt);
			 points2.push_back(kpoints2[matches1[i1].trainIdx].pt);
		  break;
      }
    }
  }
}

static void planeTest(const vector<Point2f>& points1, const vector<Point2f>& points2, 
							 vector<Point2f>& out1, vector<Point2f>& out2, int threshold){
  const int p1Size = points1.size();
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
		  continue;
      }
      if(i2 == i1)
		  continue;
      curr++;
      if(curr >= threshold){
		  out1.push_back(p11);
		  out2.push_back(p21);
		  break;
      }
    }
  }
}

Mat affine3;
bool CpuFeaturesMatcher::match(const mobots_msgs::FeatureSet& img1, const mobots_msgs::FeatureSet& img2, geometry_msgs::Pose2D& delta) const{  
  moduleStarted("cpu matcher + get transform");
  vector<DMatch> matches1;
  vector<DMatch> matches2;
  vector<Point2f> points1;
  vector<Point2f> points2;
  vector<Point2f> points1Refined;
  vector<Point2f> points2Refined;
  
  
  matcher->match(img1.descriptors, img2.descriptors, matches1);
  matcher->match(img2.descriptors, img1.descriptors, matches2);
  cout << "matches: " << matches1.size() << endl;
  symmetryTest(matches1, matches2, img1.keyPoints, img2.keyPoints, points1, points2);
  cout << "symmetric matches: " << points1.size() << endl;
  
  int threshold = 4;
  int lastSize = -1;
  for(int i = 0; true; i++){
	 cout << "round " << i << "size " << points1.size() << endl;
	 planeTest(points1, points2, points1Refined, points2Refined, threshold);
	 int size = points1.size();
	 if(size == lastSize)
		break;
	 if(size < 3)
		return false;
	 lastSize = size;
	 points1 = points1Refined;
	 points2 = points2Refined;
	 points1Refined.clear();
	 points2Refined.clear();
  }

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
  Mat d = estimateRigidTransform(points2Refined, points1Refined, false);
  cout << "size after homo " << points1Refinedx2.size() << endl;
  affine3 = d;
  /*cout << "new d " << endl << d << endl;
  affine3 = d;
  d = estimateRigidTransform(points2Refined, a, true);
  cout << "new d with true" << endl << d << endl;*/
  if(abs(d.at<double>(0,0)) > 2
	 || abs(d.at<double>(0,1)) > 2
	 || abs(d.at<double>(1,0)) > 2
	 || abs(d.at<double>(1,1)) > 2){
		cerr << "faulty matrix detected!! : " << endl << d << endl;
		//return false;
	 }
  delta.theta = -atan2(-d.at<double>(0,1), d.at<double>(0,0));
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
  drawing::drawMatches2(gimage1, points1, gimage2, points2,
               img_matches, Scalar::all(-1), Scalar::all(-1),
               DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, 50);
  imshow("good Matches", img_matches);
  /*Mat matches3;
  drawing::drawMatches(gimage1, img1.keyPoints, gimage2, img2.keyPoints,
               good_matches, matches3, Scalar::all(-1), Scalar::all(-1),
               mask2, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, 50);
  imshow("real ror Matches", matches3);*/
  return true;
}

/*
 * == unused old == */

/*
 * from opencv cookbook
 *//*
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
}*/
