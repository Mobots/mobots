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
		cout << "usage: " << argv[0] << "matrixfile dir outputdir imagecount (starts by 1)" << endl;
		return 1;
	}
	FileStorage fs(argv[1], FileStorage::READ);
	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoefficients;
	fs.release();
	/*cameraMatrix.create(3, 3, CV_64F);
	cameraMatrix.at<double>(0,0) = 3.9159612622738707e+04;
	cameraMatrix.at<double>(0,1) = 0;
	cameraMatrix.at<double>(0,2) = 3.1950000000000000e+02;
	cameraMatrix.at<double>(1,0) = 0;
	cameraMatrix.at<double>(1,1) = 3.9159612622738707e+04;
	cameraMatrix.at<double>(1,2) = 2.3950000000000000e+02;
	cameraMatrix.at<double>(2,0) = 0;
	cameraMatrix.at<double>(2,1) = 0;
	cameraMatrix.at<double>(2,2) = 1;
	
	distCoefficients.create(5, 1, CV_64F);
	distCoefficients.at<double>(0,0) = -6.7856726339458490e+01;
	distCoefficients.at<double>(0,1) = -2.8432226670911831e+02;
	distCoefficients.at<double>(0,2) = 0;
	distCoefficients.at<double>(0,3) = 0;
	distCoefficients.at<double>(0,4) = 1.4055500063605730e+04;*/
	
	cout << "camera" << endl << cameraMatrix << endl << "distortion coeffs" << endl << distCoefficients << endl;
	
	
	int count = atoi(argv[4]);
	cout << "count " << count << endl;
	for(int i = 1; i <= count; i++){
		stringstream ss1;
		ss1 << argv[2] << "/" << i << ".png";
		Mat map1, map2;
		cout << ss1.str();
		Mat img = imread(ss1.str(), 1);
		Mat newCamera = getOptimalNewCameraMatrix(cameraMatrix, distCoefficients, img.size(), 1, img.size(), 0);
		initUndistortRectifyMap(cameraMatrix, distCoefficients, Mat(), newCamera, img.size(),  CV_32FC1, map1, map2);

		Mat outImg;
		remap(img, outImg, map1, map2, INTER_LINEAR);
		stringstream ss;
		ss << argv[3] << "/" << i << ".png";
		cout << "-> " << ss.str() << endl;
		imwrite(ss.str(), outImg);
	}

}