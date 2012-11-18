#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

Mat cameraMatrix;
Mat distCoefficients;
Mat PerViewReprojectionErrors;
Mat extrinsicParameters;

int main(int argc, char** argv){
	if(argc != 5){
		cout << "usage: " << argv[0] << "dir outputdir brightness imagecount (starts by 1)" << endl;
		return 1;
	}
	int count = atoi(argv[4]);
	char brightness = atoi(argv[3]);
	for(int i = 1; i <= count; i++){
		stringstream ss1;
		ss1 << argv[1] << "/" << i << ".png";
		Mat map1, map2;
		cout << ss1.str();
		Mat img = imread(ss1.str(), 1);

		Mat outImg;
		cvtColor(img, outImg, CV_RGB2HSV);
		for(int row = 0; row < outImg.rows; row++){
			for(int col = 0; col < outImg.cols; col++){
				outImg.at<Vec3b>(row, col)[0] = brightness;
			}
		}
		cvtColor(outImg, outImg, CV_HSV2RGB);
		stringstream ss;
		ss << argv[2] << "/" << i << ".png";
		cout << "-> " << ss.str() << endl;
		imwrite(ss.str(), outImg);
	}

}