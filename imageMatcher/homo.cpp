#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
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

  SurfFeatureDetector detector(1000.0);
  std::vector<KeyPoint> keypoints1;
  std::vector<KeyPoint>keypoints2;
  detector.detect(img1, keypoints1);
  detector.detect(img2, keypoints2);
  keypoints1.erase(keypoints1.begin()+50, keypoints1.end());
  keypoints2.erase(keypoints2.begin()+50, keypoints2.end());
  
  SurfDescriptorExtractor extractor(30, 30, true);
  Mat descriptors1;
  Mat descriptors2;
  extractor.compute(img1, keypoints1, descriptors1);
  extractor.compute(img2, keypoints2, descriptors2);
  
  BruteForceMatcher<L2<float> > matcher;
  vector<DMatch> matches;
  matcher.match(descriptors1, descriptors2, matches);
  
  for(int i = 0; i < matches.size(); i++){
    std::cout << "distance " << matches[i].distance << std::endl;
  }
  
  Point2f points1[3];
  Point2f points2[3];
  for(int i = 0; i < 3; i++){
    points1[i] = keypoints1[matches[i].queryIdx].pt;
    points2[i] = keypoints1[matches[i].trainIdx].pt;
  }
  Mat result = getAffineTransform(points1, points2);
  std::cout << "Mat " << result << std::endl;
  Mat img_matches;
  drawMatches(img1, keypoints1, img2, keypoints2,
               matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Show detected matches
  imshow( "Good Matches", img_matches );
  waitKey(0);
  return 0;
}
