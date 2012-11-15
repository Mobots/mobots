#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/console.h>
#include <geometry_msgs/Pose2D.h>
#include "test/draw.h"
#include "profile.h"
#include "mobots_msgs/FeatureSet.h"
#include "feature_detector/FeaturesMatcher.h"
#include "feature_detector/FeaturesFinder.h"
#include "gpc/gpc.h"
#include "gpc/gpc.c"

using namespace std;
using namespace cv;

//extern Mat gimage1; //DEBUG
//extern Mat gimage2;
Mat H;

const char CpuFeaturesMatcher::SURF_DEFAULT[] = "FlannBased";
const char CpuFeaturesMatcher::ORB_DEFAULT[] = "BruteForce-Hamming";
static const double LENGTHDIFF_THRESHOLD = 2.0;    //if abs(distance(a) - distance(b)) > => outlier
static const double MAX_ALLOWED_MATCH_DISTANCE = 600;
static const char* TAG = "[FeaturesMatcher] ";
static const double minVarXY = 2;
static const double minVarTheta = 0.002;
static const double maxVarXY = 100;
static const double maxVarTheta = 0.35;


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
//  Mat gimage1;
//  Mat gimage2;
Mat mm1, mm2;
Mat mega;
Mat kpoints1;
Mat kpoints2;
Mat good_matches;
Mat good_matches_r;
Mat homo;

// void getIntersectionRois(const geometry_msgs::Pose2D& delta, Mat& roi1, Mat& roi2){
// 	gpc_vertex verticesA[4];
// 	gpc_vertex verticesB[4];
// 	gpc_vertex_list listA;
// 	gpc_vertex_list listB;
// 	listA.num_vertices = 4;
// 	listB.num_vertices = 4;
// 	listA.vertex = verticesA;
// 	listB.vertex = verticesB;
// 	const int halfWidth = gimage1.cols/2;
// 	const int halfHeight = gimage1.rows/2;
// 	verticesA[0] = {-halfWidth, +halfHeight};
// 	verticesA[1] = {+halfWidth, +halfHeight};
// 	verticesA[2] = {+halfWidth, -halfHeight};
// 	verticesA[3] = {-halfWidth, -halfHeight};
// 	
// 	double cost = cos(delta.theta);
// 	double sint = sin(delta.theta);
// 	for(int i = 0; i < 4; i++){
// 		double x = verticesA[i].x;
// 		double y = verticesA[i].y;
// 		verticesB[i].x = x*cost - sint*y + delta.x;
// 		verticesB[i].y = x*sint + cost*y + delta.y;
// 	}
// 	
// 	gpc_polygon rectA, rectB, rectResult;
// 	int hole = 0;
// 	rectA.hole = &hole;
// 	rectA.num_contours = 1;
// 	rectA.contour = &listA;
// 	
// 	rectB.hole = &hole;
// 	rectB.num_contours = 1;
// 	rectB.contour = &listB;
// 	
//  	gpc_polygon_clip(GPC_INT, &rectA, &rectB, &rectResult);
// 	int resultListCount = rectResult.contour->num_vertices;
// 	cv::Point* cvPointsA = new cv::Point[resultListCount];
// 	cv::Point* cvPointsB = new cv::Point[resultListCount];
// 	gpc_vertex_list* resultList = rectResult.contour;
// 	for(int i = 0; i < resultListCount; i++){
// 		cvPointsA[i].x = resultList->vertex[i].x;
// 		cvPointsA[i].y = resultList->vertex[i].y;
// 		
// 		cvPointsB[i].x =  cvPointsA[i].x*cost + cvPointsA[i].y*sint - delta.x + halfWidth;
// 		cvPointsB[i].y = -cvPointsA[i].x*sint + cvPointsA[i].y*cost - delta.y + halfHeight;
// 		
// 		cvPointsA[i].x += halfWidth;
// 		cvPointsA[i].y += halfHeight;
// 	}
// 	roi1 = Mat::ones(gimage1.size(), CV_8UC1);
// 	roi2 = Mat::ones(gimage1.size(), CV_8UC1);
// 	cv::fillConvexPoly(roi1, cvPointsA, resultListCount, 255);
//   cv::fillConvexPoly(roi2, cvPointsB, resultListCount, 255);
//   mm1 = gimage1.clone();
//   mm2 = gimage2.clone();
//   cv::fillConvexPoly(mm1, cvPointsA, resultListCount, 255);
//   cv::fillConvexPoly(mm2, cvPointsB, resultListCount, 255);
// }

bool CpuFeaturesMatcher::match(const FeatureSet& img1, const FeatureSet& img2, MatchResult& result) const{  
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
  symmetryTest(matches1, matches2, img1.keyPoints, img2.keyPoints, points1, points2);
  /*gimage1 = image1;
	gimage2 = image2;
  cv::drawKeypoints(image1, img1.keyPoints, kpoints1);
  cv::drawKeypoints(image2, img2.keyPoints, kpoints2);*/
  
  int threshold = 4;
  int lastSize = -1;
  for(int i = 0; true; i++){
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
	if(points1Refined.size() >= 10){
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
		cout << "symmetric " << points1.size() << " size after plane " << points1Refined.size() << " and  after homo " << points1Refinedx2.size() << endl;		
	}else{
		return false;
	}
  if(abs(transformMatrix.at<double>(0,0)) > 1.1
	 || abs(transformMatrix.at<double>(0,1)) > 1.1
	 || abs(transformMatrix.at<double>(1,0)) > 1.1
	 || abs(transformMatrix.at<double>(1,1)) > 1.1){
	 ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": faulty matrix detected (&rejected)!! : \n" << transformMatrix);
		return false;
  }


  result.delta.theta = atan2(-transformMatrix.at<double>(0,1), transformMatrix.at<double>(0,0));
	if(result.delta.theta < 0)
		result.delta.theta += 2*M_PI;
	
	result.delta.x = transformMatrix.at<double>(0,2);
	result.delta.y = transformMatrix.at<double>(1,2);
	
// 	  Mat roi1, roi2;
//   getIntersectionRois(result.delta, roi1, roi2);
// 	
// 	 Mat hsv1, hsv2, hsv3, hsv4;
// 	//cout << mg1 << endl;
// 	//cout << "cols " << mg1.cols << " rows " << mg1.rows << endl;
// 	MatND hist1, hist2, hist3, hist4;
// 	cvtColor(gimage1, hsv1, CV_BGR2HSV);
// 	cvtColor(gimage2, hsv2, CV_BGR2HSV);
// 	cvtColor(gimage1, hsv3, CV_BGR2HSV);
// 	cvtColor(gimage2, hsv4, CV_BGR2HSV);
// 	// Using 30 bins for hue and 32 for saturation
// 	int h_bins = 50; int s_bins = 60;
// 	int histSize[] = { h_bins, s_bins , 50};
// 	// hue varies from 0 to 256, saturation from 0 to 180
// 	float h_ranges[] = { 0, 256 };
// 	float s_ranges[] = { 0, 180 };
// 
// 	const float* ranges[] = { h_ranges, s_ranges, h_ranges};
// 	// Use the o-th and 1-st channels
// 	int channels[] = { 0, 1, 2 };
// 	
// 	calcHist( &hsv1, 1, channels, roi1, hist1, 3, histSize, ranges, true, false );
// 	normalize( hist1, hist1);
// 	calcHist( &hsv2, 1, channels, roi2, hist2, 3, histSize, ranges, true, false );
// 	normalize( hist2, hist2);
// 	double base_base = compareHist( hist1, hist2, 0 );
// 	cout << "histogram similarity with rois: " << base_base << endl;
// 	double norm = norm(hist1, hist2);
// 	cout << "norm " << norm << endl;
// 	
// 	/*calcHist( &hsv1, 1, channels, Mat(), hist3, 2, histSize, ranges, true, false );
// 	//normalize( hist3, hist3);
// 	calcHist( &hsv2, 1, channels, Mat(), hist4, 2, histSize, ranges, true, false );
// 	//normalize( hist4, hist4);
// 	double base_base2 = compareHist( hist3, hist4, 0 );
// 	cout << "histogram similarity without rois: " << base_base2 << endl;*/
	
	double confidence = 150/points1Refinedx2.size();
	confidence = confidence*confidence;
	if(confidence < 1)
		confidence = 1;
	result.varX = minVarXY*confidence;
	result.varY = minVarXY*confidence;
	result.varTheta = minVarTheta*confidence;
	if(result.varX > maxVarXY)
		result.varX = maxVarXY;
	if(result.varY > maxVarXY)
		result.varY = maxVarXY;
	if(result.varTheta > maxVarTheta)
		result.varTheta = maxVarTheta;
	moduleEnded();
  
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
