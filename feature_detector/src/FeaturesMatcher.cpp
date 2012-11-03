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
#include <boost/concept_check.hpp>

using namespace std;
using namespace cv;

//extern Mat gimage1; //DEBUG
//extern Mat gimage2;
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
extern Mat gimage1;
extern Mat gimage2;
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
		transformMatrix = estimateRigidTransform(points2Refinedx2, points1Refinedx2, false);
		//transformMatrix = homo;
		cout << "size after homo " << points1Refinedx2.size() << endl;		
		  /*drawing::drawMatches2(gimage1, points1Refinedx2, gimage2, points2Refinedx2,
               good_matches_r, Scalar::all(-1), Scalar::all(-1),
               DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, 50);*/
	}else{
		transformMatrix = estimateRigidTransform(points2Refined, points1Refined, false);
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
  
  //Mat roi1;
  //Mat roi2;
  //getIntersectionRois(delta, roi1, roi2);
  
  /*Mat hsv1;
  Mat hsv2;
  MatND hist1, hist2, hist3, hist4;
  cvtColor(image1, hsv1, CV_BGR2HSV);
  cvtColor(image2, hsv2, CV_BGR2HSV);
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
  
  drawing::drawMatches2(gimage1, points1Refinedx2, gimage2, points2Refinedx2,
               good_matches_r, Scalar::all(-1), Scalar::all(-1),
               DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, 50);*/



  return true;
}

Mat mm1, mm2;
Mat mega;



using namespace boost::assign;

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point > polygon;
typedef boost::geometry::model::ring<boost::geometry::model::d2::point_xy<double> > boost_ring;

void rotatePoint(point& p, const point center, double angle){
	float s = sin(angle);
  float c = cos(angle);

  // translate point back to origin:
  p.x(p.x() - center.x());
  p.y(p.y() - center.y());

  // rotate point
  float xnew = p.x() * c + p.y() * s;
  float ynew = -p.x() * s + p.y() * c;

  // translate point back:
  p.x(xnew + center.x());
  p.y(ynew + center.y());
}
void cvBoxPoints(Point2d center, double width, double height, double angle, boost_ring&);
bool getIntersectionRois(const geometry_msgs::Pose2D& delta, Mat& roi1, Mat& roi2){
  point vertices[5];
  polygon poly1, poly2;
  

  boost_ring ring1, ring2;
  //ring1 += point(0, 0), point(gimage1.cols, 0), point(gimage1.cols, gimage1.rows), point(0, gimage1.rows), point(0,0);
  ring1 += point(0, 0), point(gimage1.cols, 0), point(gimage1.cols, gimage1.rows), point(0, gimage1.rows), point(0,0);
	const double cost = cos(delta.theta);
	const double sint = sin(delta.theta);
	double centerX = gimage1.cols/2 ;
	double centerY = gimage1.rows/2 ;
	point pointsa[4];
	double pointsdata[5][2];
	double pointsdata1[5][2];
	/*pointsa[0] = point(-centerX, -centerY);
	pointsa[1] = point(-centerX, centerY);
	pointsa[2] = point(centerX, centerY);
	pointsa[3] = point(centerX, -centerY);
	/*pointsdata[0][0] = 0;
	pointsdata[0][1] = 0;
	pointsdata[1][0] = 0;
	pointsdata[1][1] = gimage1.rows;
	pointsdata[2][0] = gimage1.cols;
	pointsdata[2][1] = gimage1.rows;
	pointsdata[3][0] = gimage1.cols;
	pointsdata[3][1] = 0;
	memcpy(pointsdata1, pointsdata, 10*sizeof(double));*/
point pointsc[4];
	pointsa[3] = point(0, 0);
	pointsa[2] = point(0, gimage1.rows);
	pointsa[1] = point(gimage1.cols, gimage1.rows);
	pointsa[0] = point(gimage1.cols, 0);
	pointsc[3] = point(0, 0);
	pointsc[2] = point(0, gimage1.rows);
	pointsc[1] = point(gimage1.cols, gimage1.rows);
	pointsc[0] = point(gimage1.cols, 0);
	//boost::geometry::correct(ring2);
	for(int i = 0; i< 4; i++){

		/*pointsa[i].x(x*cost + y*sint + delta.x + centerX);
		pointsa[i].y(-x*sint + y*cost + delta.y + centerY);*/

		rotatePoint(pointsa[i], point(centerX, centerY), delta.theta);
		cout << "(" << pointsc[i].x() << "," << pointsc[i].y() << ") => (" << pointsa[i].x() << "," << pointsa[i].y() << ")";
		cout << "with center (" << centerX << "," << centerY << ") with theta = " << delta.theta << endl;
		double x = pointsa[i].x();
		double y = pointsa[i].y();
		pointsa[i].x(x + delta.x);
		pointsa[i].y(y + delta.y);
		cout << "x " << pointsa[i].x() << " y " << pointsa[i].y() << endl;
	}
	cv::Point* cvPoints3 = new cv::Point[5];
  for(int i = 0; i < 5; i++){
	 cvPoints3[i].x = pointsa[i].x();
	 cvPoints3[i].y = pointsa[i].y();
  }
	for(int i = 0; i < 5; i++){
		ring1.at(i).x(ring1.at(i).x());
		ring1.at(i).y(ring1.at(i).y());
  }
  mega.create(700, 700, CV_8UC1);
  cv::fillConvexPoly(mega, cvPoints3, 5, cv::Scalar(192));
	/*for(int i = 0; i< 4; i++){
		double x = pointsdata[i][0] - centerX;
		double y = pointsdata[i][1] - centerY;
		pointsdata[i][0] = x*cost - y*sint + delta.x + centerX;
		pointsdata[i][1] = x*sint + y*cost + delta.y + centerY;
	}
	polygon poly3;
	boost::geometry::append(poly3, pointsdata);
	polygon poly4;
	boost::geometry::append(poly4, pointsdata1);*/
	ring2 += pointsa[0], pointsa[1], pointsa[2], pointsa[3], pointsa[0];
	/*vertices[0][0] = 0;
  vertices[0][1] = 0;
  vertices[4][0] = 0;
  vertices[4][1] = 0;
  
  vertices[1][0] = gimage1.cols;
  vertices[1][1] = 0;
	const double cost = cos(delta.theta);
  const double sint = sin(delta.theta);
  
  vertices[2][0] = gimage1.cols;
  vertices[2][1] = gimage1.rows;
  
  vertices[3][0] = 0;
  vertices[3][1] = gimage1.rows;*/
  //double a[5][2];
  //boost::geometry::assign(poly1, a);
  
  //cvBoxPoints(Point2d(gimage1.cols/2+delta.x, gimage1.rows/2+delta.y), gimage1.rows, gimage1.cols, delta.theta, ring2);
  //vertices[4][0] = vertices[0][0];
  //vertices[4][1] = vertices[0][1];
  //boost::geometry::append(poly2, vertices);
  boost::geometry::correct(ring1);
  boost::geometry::correct(ring2);

  std::deque<polygon > resultList;
  boost::geometry::intersection(ring1, ring2, resultList);
  if(resultList.empty()){
	 cout << "empty list " << endl;
	 return false;
  }
  roi1 = Mat::zeros(gimage1.size(), CV_8UC1);
  roi2 = Mat::zeros(gimage1.size(), CV_8UC1);
  polygon result = resultList.front();
  const vector<point> points = result.outer();
  cv::Point* cvPoints = new cv::Point[points.size()];
  cv::Point* cvPoints2 = new cv::Point[points.size()];
  for(int i = 0; i < points.size(); i++){
	 cvPoints[i].x = points[i].x() -delta.x;
	 cvPoints[i].y = points[i].y() +delta.y;
	 
	 cout << "x1 " << cvPoints[i].x << " y1 " << cvPoints[i].y << endl;
  }
	for(int i = 0; i < points.size(); i++){
	 
	 cvPoints2[i].x = points[i].x()*cost + points[i].y()*sint - delta.x;
	 cvPoints2[i].y = -points[i].x()*sint + points[i].y()*cost - delta.y;
	 cout << "x2 " << cvPoints2[i].x << " y2 " << cvPoints2[i].y << endl;
  }
  cv::fillConvexPoly(roi1, cvPoints, points.size(), cv::Scalar(1));
  cv::fillConvexPoly(roi2, cvPoints2, points.size(), cv::Scalar(1));
  mm1 = gimage1.clone();
  mm2 = gimage2.clone();
  cv::fillConvexPoly(mm1, cvPoints, points.size(), cv::Scalar(184));
  cv::fillConvexPoly(mm2, cvPoints2, points.size(), cv::Scalar(192));
  return true;
}

/**
 * copied from old opencv
 */
void cvBoxPoints(Point2d center, double width, double height, double angle, boost_ring& ring){
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
