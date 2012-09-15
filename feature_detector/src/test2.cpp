#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching/stitcher.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv){
  Mat img1 = imread(argv[1]);
  Mat img2 = imread(argv[2]);
  Stitcher stitcher = Stitcher::createDefault();
  Mat result;
  vector<Mat> images;
  images.push_back(img1);
  images.push_back(img2);
  stitcher.stitch(images, result);
  imshow("result", result);
  waitKey();
}