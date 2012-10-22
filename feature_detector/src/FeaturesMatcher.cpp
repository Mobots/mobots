#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/console.h>
#include <geometry_msgs/Pose2D.h>

#include <boost/geometry.hpp> //for intersection calculation
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>
#include <boost/assign.hpp>

#include "draw.h"
#include "profile.h"
#include "mobots_msgs/FeatureSet.h"
#include "feature_detector/FeaturesMatcher.h"
#include "feature_detector/FeaturesFinder.h"

using namespace std;
using namespace cv;

extern Mat gimage1; //DEBUG
extern Mat gimage2;
Mat H;

const char CpuFeaturesMatcher::SURF_DEFAULT[] = "FlannBased";
const char CpuFeaturesMatcher::ORB_DEFAULT[] = "BruteForce-Hamming";
static const double LENGTHDIFF_THRESHOLD = 1.0;    //if abs(distance(a) - distance(b)) > => outlier
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
Mat affine2;
bool getIntersectionRois(const geometry_msgs::Pose2D&, Mat&, Mat&);
bool CpuFeaturesMatcher::match(const FeatureSet& img1, const FeatureSet& img2, geometry_msgs::Pose2D& delta) const{  
  moduleStarted("cpu matcher + get transform");
  vector<DMatch> matches1;
  vector<DMatch> matches2;
  vector<Point2f> points1;
  vector<Point2f> points2;
  vector<Point2f> points1Refined;
  vector<Point2f> points2Refined;
  //cout << img1.keyPoints.size() << " " << img1.descriptors.size() << cout << img2.keyPoints.size() << " " << img2.descriptors.size() << endl;
  
  
  matcher->match(img1.descriptors, img2.descriptors, matches1);
  matcher->match(img2.descriptors, img1.descriptors, matches2);
  cout << "matches: " << matches1.size() << endl;
  symmetryTest(matches1, matches2, img1.keyPoints, img2.keyPoints, points1, points2);
  cout << "symmetric matches: " << points1.size() << endl;
  
  Mat img_matches;
  cv::drawKeypoints(gimage1, img1.keyPoints, img_matches);
  imshow("keypoints 1", img_matches);
  cv::drawKeypoints(gimage2, img2.keyPoints, img_matches);
  imshow("keypoints 2", img_matches);
  
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
	/* points1Refined.clear();
	 points2Refined.clear();
  threshold = 20;
  lastSize = -1;
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
  affine2 = estimateRigidTransform(points2Refinedx2, points1Refinedx2, false);
  /*cout << "new d " << endl << d << endl;
  affine3 = d;
  d = estimateRigidTransform(points2Refined, a, true);
  cout << "new d with true" << endl << d << endl;*/
  if(abs(d.at<double>(0,0)) > 1.1
	 || abs(d.at<double>(0,1)) > 1.1
	 || abs(d.at<double>(1,0)) > 1.1
	 || abs(d.at<double>(1,1)) > 1.1){
	 ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": faulty matrix detected (&rejected)!! : \n" << d);
		//return false;
  }
  delta.theta = -atan2(-d.at<double>(0,1), d.at<double>(0,0));
  delta.x = d.at<double>(0,2);
  delta.y = d.at<double>(1,2);
  
  Mat roi1;
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
  moduleEnded();
  
  drawing::drawMatches2(gimage1, points1Refined, gimage2, points2Refined,
               img_matches, Scalar::all(-1), Scalar::all(-1),
               DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, 50);
  imshow("good Matches", img_matches);
  drawing::drawMatches2(gimage1, points1Refinedx2, gimage2, points2Refinedx2,
               img_matches, Scalar::all(-1), Scalar::all(-1),
               DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, 50);
  imshow("good Matches with ransac", img_matches);
  /*Mat matches3;
  drawing::drawMatches(gimage1, img1.keyPoints, gimage2, img2.keyPoints,
               good_matches, matches3, Scalar::all(-1), Scalar::all(-1),
               mask2, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, 50);
  imshow("real ror Matches", matches3);*/
  return true;
}

using namespace boost::assign;

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point > polygon;
typedef boost::geometry::model::ring<boost::geometry::model::d2::point_xy<double> > ring;
void cvBoxPoints(Point2d center, double width, double height, double angle, ring&);
bool getIntersectionRois(const geometry_msgs::Pose2D& delta, Mat& roi1, Mat& roi2){
  point vertices[5];
  polygon poly1, poly2;
  

  boost::geometry::model::ring<boost::geometry::model::d2::point_xy<double> > ring1, ring2;
  //ring1 += point(0, 0), point(gimage1.cols, 0), point(gimage1.cols, gimage1.rows), point(0, gimage1.rows), point(0,0);
  ring1 += point(0, 0), point(0, gimage1.rows), point(gimage1.cols, gimage1.rows), point(gimage1.cols, 0), point(0,0);
  /*vertices[0][0] = 0;
  vertices[0][1] = 0;
  vertices[4][0] = 0;
  vertices[4][1] = 0;
  
  vertices[1][0] = gimage1.cols;
  vertices[1][1] = 0;
  
  vertices[2][0] = gimage1.cols;
  vertices[2][1] = gimage1.rows;
  
  vertices[3][0] = 0;
  vertices[3][1] = gimage1.rows;*/
  //double a[5][2];
  //boost::geometry::assign(poly1, a);
  
  cvBoxPoints(Point2d(gimage1.cols/2+delta.x, gimage1.rows/2+delta.y), gimage1.rows, gimage1.cols, delta.theta, ring2);
  //vertices[4][0] = vertices[0][0];
  //vertices[4][1] = vertices[0][1];
  //boost::geometry::append(poly2, vertices);
  //boost::geometry::correct(ring1);
  //boost::geometry::correct(ring2);
  std::deque<polygon > resultList;
  boost::geometry::intersection(ring1, ring2, resultList);
  if(resultList.empty()){
	 cout << "empty list " << endl;
	 return false;
  }
  roi1 = Mat::zeros(gimage1.size(), CV_8UC1);
  roi2 = Mat::zeros(gimage1.size(), CV_8UC1);
  polygon result = resultList.front();
  vector<point> points = result.outer();
  cv::Point* cvPoints = new cv::Point[points.size()];
  cv::Point* cvPoints2 = new cv::Point[points.size()];
  double cost = cos(delta.theta);
  double sint = sin(delta.theta);
  for(int i = 0; i < points.size(); i++){
	 cvPoints[i].x = boost::geometry::get<0>(points[i]);
	 cvPoints[i].y = boost::geometry::get<1>(points[i]);
	 
	 cvPoints2[i].x = boost::geometry::get<0>(points[i])*cost + boost::geometry::get<1>(points[i])*sint - delta.x;
	 cvPoints2[i].y = -boost::geometry::get<0>(points[i])*sint + boost::geometry::get<1>(points[i])*cost - delta.y;
	 //cvPoints2[i].x = boost::geometry::get<0>(points[i]) - delta.x;
	 //cvPoints2[i].y = boost::geometry::get<1>(points[i]) - delta.y;
	 cout << "x " << cvPoints2[i].x << " y " << cvPoints2[i].y << endl;
  }
  cv::fillConvexPoly(roi1, cvPoints, points.size(), cv::Scalar(1));
  cv::fillConvexPoly(roi2, cvPoints2, points.size(), cv::Scalar(1));
  Mat m1 = gimage1.clone();
  Mat m2 = gimage2.clone();
  cv::fillConvexPoly(m1, cvPoints, points.size(), cv::Scalar(1));
  cv::fillConvexPoly(m2, cvPoints2, points.size(), cv::Scalar(1));
  imshow("m1", m1);
  imshow("m2", m2);
  return true;
}

/**
 * copied from old opencv
 */
void cvBoxPoints(Point2d center, double width, double height, double angle, ring& ring){
    float a = (float)cos(angle)*0.5f;
    float b = (float)sin(angle)*0.5f;
	 double x0 = center.x - a*height - b*width;
	 double y0 = center.y + b*height - a*width;
	 double x1 = center.x + a*height - b*width;
	 double y1 = center.y - b*height - a*width;
	 
	 /*ring += point(x0, y0);
	 ring += point(x1, y1);
	 ring += point(2*center.x - x0, 2*center.y - y0);
	 ring += point(2*center.x - x1, 2*center.y - y1);
	 ring += point(x0, y0);*/
	 ring += point(x0, y0);
	 ring += point(2*center.x - x1, 2*center.y - y1);
	 ring += point(2*center.x - x0, 2*center.y - y0);
	 ring += point(x1, y1);
	 ring += point(x0, y0);
	 
	 /*pt[0][0] = center.x - a*height - b*width;
    pt[0][1] = center.y + b*height - a*width;
    pt[1][0] = center.x + a*height - b*width;
    pt[1][1] = center.y - b*height - a*width;
    pt[2][0] = 2*center.x - pt[0][0];
    pt[2][1] = 2*center.y - pt[0][1];
    pt[3][0] = 2*center.x - pt[1][0];
    pt[3][1] = 2*center.y - pt[1][1];*/
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
