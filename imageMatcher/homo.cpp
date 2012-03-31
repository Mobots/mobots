#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <stdio.h>
using namespace cv;

int* compute(Mat&, Mat&);

int main(int argc, char **argv){
  Mat img1 = imread(argv[1], 1);
  Mat img2 = imread(argv[2], 1);
  int *result = compute(img1, img2);
  /*cout << "transx " << result[0] << endl;
  cout << "transy " << result[1] << endl;
  cout << "rot " << result[2] << endl;*/
  
}


int* compute(Mat &img1, Mat &img2){

  SurfFeatureDetector detector;
  std::vector<KeyPoint> keypoints1;
  std::vector<KeyPoint>keypoints2;
  detector.detect(img1, keypoints1);
  detector.detect(img2, keypoints2);
  /*keypoints1.erase(keypoints1.begin()+50, keypoints1.end());
  keypoints2.erase(keypoints2.begin()+50, keypoints2.end());*/
  
  SurfDescriptorExtractor extractor;
  Mat descriptors1;
  Mat descriptors2;
  extractor.compute(img1, keypoints1, descriptors1);
  extractor.compute(img2, keypoints2, descriptors2);
  

  
  FlannBasedMatcher matcher;
  vector<DMatch> matches;
  matcher.match(descriptors1, descriptors2, matches);
  double min_dist = 100, max_dist = 0;
  for( int i = 0; i < descriptors1.rows; i++ ){ 
    double dist = matches[i].distance;
    if( dist < min_dist && dist > 0 ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );
  std::vector< DMatch > good_matches;
  for( int i = 0; i < descriptors1.rows; i++ ){ 
    if(matches[i].distance < 5*min_dist)
     good_matches.push_back(matches[i]); 
  }
  printf("match count %lu\n", good_matches.size());  
  std::vector<Point2f> points1;
  std::vector<Point2f> points2;
  for(int i = 0; i < good_matches.size(); i++){
    points1.push_back(keypoints1[good_matches[i].queryIdx].pt);
    points2.push_back(keypoints2[good_matches[i].trainIdx].pt);
  }
  for(int i = 0; i < 4; i++){
    std::cout << points1[i] << std::endl;
  }
  std::cout << std::endl;
  for(int i = 0; i < 4; i++){
    std::cout << points2[i] << std::endl;
  }
  //Mat result = getAffineTransform( points1, points2);
  Mat result = findHomography(points1, points2, CV_RANSAC, 3);
  std::cout << "Mat " << result << std::endl;
  Mat img_matches;
  drawMatches(img1, keypoints1, img2, keypoints2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), 0);

  //-- Show detected matches
  imshow( "Good Matches", img_matches );
  waitKey(0);
  /*int *result2 = new int[3];
  result2[0] = result.at<float>(0, 2);
  result2[1] = result.at<float>(1, 2);*/
}
