#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/console.h>
#include <geometry_msgs/Pose2D.h>
#include "draw.h"
#include "profile.h"
#include "mobots_msgs/FeatureSet.h"
#include "feature_detector/FeaturesMatcher.h"
#include "feature_detector/FeaturesFinder.h"

using namespace std;
using namespace cv;

//extern Mat gimage1; //DEBUG
//extern Mat gimage2;
Mat H;

const char CpuFeaturesMatcher::SURF_DEFAULT[] = "FlannBased";
const char CpuFeaturesMatcher::ORB_DEFAULT[] = "BruteForce-Hamming";
static const double LENGTHDIFF_THRESHOLD = 1.0;    //if abs(distance(a) - distance(b)) > => outlier
static const double MAX_ALLOWED_MATCH_DISTANCE = 600;
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
		  matches1[i1].trainIdx /*&&
		  matches1[i1].distance < MAX_ALLOWED_MATCH_DISTANCE*/){
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
Mat affine2;
//extern Mat gimage1;
//extern Mat gimage2;
Mat mm1, mm2;
Mat mega;
Mat kpoints1;
Mat kpoints2;
Mat good_matches;
Mat good_matches_r;
Mat homo;

bool getIntersectionRois(const geometry_msgs::Pose2D&, Mat&, Mat&);
bool CpuFeaturesMatcher::match(const FeatureSet& img1, const FeatureSet& img2, geometry_msgs::Pose2D& delta/*, Mat& image1, Mat& image2*/) const{  
  moduleStarted("cpu matcher + get transform");
  vector<DMatch> matches1;
  vector<DMatch> matches2;
  vector<Point2f> points1;
  vector<Point2f> points2;
  vector<Point2f> points1Refined;
  vector<Point2f> points2Refined;
	vector<Point2f> points1Refinedx2;
	vector<Point2f> points2Refinedx2;
  //cout << img1.keyPoints.size() << " " << img1.descriptors.size() << cout << img2.keyPoints.size() << " " << img2.descriptors.size() << endl;
  
  
  matcher->match(img1.descriptors, img2.descriptors, matches1);
  matcher->match(img2.descriptors, img1.descriptors, matches2);
  cout << "matches: " << matches1.size() << endl;
  symmetryTest(matches1, matches2, img1.keyPoints, img2.keyPoints, points1, points2);
  cout << "symmetric matches: " << points1.size() << endl;
  /*gimage1 = image1;
	gimage2 = image2;
  cv::drawKeypoints(image1, img1.keyPoints, kpoints1);
  cv::drawKeypoints(image2, img2.keyPoints, kpoints2);*/
  
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
	 //if(i == 1 ) break;
	 lastSize = size;
	 points1 = points1Refined;
	 points2 = points2Refined;
	 points1Refined.clear();
	 points2Refined.clear();
  }
  Mat transformMatrix;
	if(points1Refined.size() >= 7){
		vector<uchar> status;
		homo = findHomography(points2Refined, points1Refined, CV_RANSAC, 3, status);
		for(int i = 0; i < status.size(); i++){
			if(status[i]){
				points1Refinedx2.push_back(points1Refined[i]);
				points2Refinedx2.push_back(points2Refined[i]);
			}
		}
		if(points1Refinedx2.size() < 10)
			return false;
		transformMatrix = estimateRigidTransform(points2Refinedx2, points1Refinedx2, false);
		//transformMatrix = homo;
		cout << "size after homo " << points1Refinedx2.size() << endl;		
		  /*drawing::drawMatches2(gimage1, points1Refinedx2, gimage2, points2Refinedx2,
               good_matches_r, Scalar::all(-1), Scalar::all(-1),
               DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, 50);*/
	}else{
		//transformMatrix = estimateRigidTransform(points2Refined, points1Refined, false);
		return false;
	}
  /*cout << "new d " << endl << d << endl;
  affine3 = d;
  d = estimateRigidTransform(points2Refined, a, true);
  cout << "new d with true" << endl << d << endl;*/
  if(abs(transformMatrix.at<double>(0,0)) > 1.1
	 || abs(transformMatrix.at<double>(0,1)) > 1.1
	 || abs(transformMatrix.at<double>(1,0)) > 1.1
	 || abs(transformMatrix.at<double>(1,1)) > 1.1){
	 ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": faulty matrix detected (&rejected)!! : \n" << transformMatrix);
		//return false;
  }
  delta.theta = atan2(-transformMatrix.at<double>(0,1), transformMatrix.at<double>(0,0));
	if(delta.theta < 0)
		delta.theta += 2*M_PI;
  //double theta1 = atan2(-transformMatrix.at<double>(0,1), transformMatrix.at<double>(0,0));
	//double theta2 = atan2(-transformMatrix.at<double>(1,1), transformMatrix.at<double>(1,1));
	//delta.theta = (theta1-theta2)/2;
	//if(delta.theta < 0)
	//	delta.theta += 2*M_PI;
  delta.x = transformMatrix.at<double>(0,2);
  delta.y = transformMatrix.at<double>(1,2);
	moduleEnded();
  
  /*Mat roi1;
  Mat roi2;
  getIntersectionRois(delta, roi1, roi2);
  
  Mat hsv1;
  Mat hsv2;
  MatND hist1, hist2, hist3, hist4;
  cvtColor(gimage1, hsv1, CV_BGR2HSV);
  cvtColor(gimage2, hsv2, CV_BGR2HSV);
  // Using 30 bins for hue and 32 for saturation
  int h_bins = 50; int s_bins = 60;
  int histSize[] = { h_bins, s_bins };
  // hue varies from 0 to 256, saturation from 0 to 180
  float h_ranges[] = { 0, 256 };
  float s_ranges[] = { 0, 180 };

  const float* ranges[] = { h_ranges, s_ranges };
  // Use the o-th and 1-st channels
  int channels[] = { 0, 1 };
  calcHist( &hsv1, 1, channels, roi1, hist1, 2, histSize, ranges, true, false );
  normalize( hist1, hist1, 0, 1, NORM_MINMAX, -1, Mat() );
  calcHist( &hsv2, 1, channels, roi2, hist2, 2, histSize, ranges, true, false );
  normalize( hist2, hist2, 0, 1, NORM_MINMAX, -1, Mat() );
  double base_base = compareHist( hist1, hist2, 0 );
  cout << "histogram similarity with rois: " << base_base << endl;
  calcHist( &hsv1, 1, channels, Mat(), hist3, 2, histSize, ranges, true, false );
  normalize( hist1, hist1, 0, 1, NORM_MINMAX, -1, Mat() );
  calcHist( &hsv2, 1, channels, Mat(), hist4, 2, histSize, ranges, true, false );
  normalize( hist2, hist2, 0, 1, NORM_MINMAX, -1, Mat() );
  double base_base2 = compareHist( hist3, hist4, 0 );
  cout << "histogram similarity without rois: " << base_base2 << endl;
  moduleEnded();*/
  /*if(points1Refinedx2.empty())
		  drawing::drawMatches2(gimage1, points1Refined, gimage2, points2Refined,
               good_matches_r, Scalar::all(-1), Scalar::all(-1),
               DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, 50);
	else
		drawing::drawMatches2(gimage1, points1Refinedx2, gimage2, points2Refinedx2,
               good_matches_r, Scalar::all(-1), Scalar::all(-1),
               DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, 50);*/



  return true;
}
