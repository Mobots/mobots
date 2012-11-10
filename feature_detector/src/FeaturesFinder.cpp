#include <opencv2/gpu/gpumat.hpp>
#include <iostream>

#include <opencv2/features2d/features2d.hpp>
#include "feature_detector/FeaturesFinder.h"
#include "profile.h"

using namespace std;
using namespace cv;

Ptr<FeaturesFinder> FeaturesFinder::getDefault(){
  return new OrbFeaturesFinder(600, 5);
  //return new SurfFeaturesFinder;
}

OrbFeaturesFinder::OrbFeaturesFinder(int nfeatures, int sliceCount)
  :sliceCount(sliceCount){
    orb = new cv::ORB(nfeatures, 1.2f, 8, 0, 0, 2, cv::ORB::HARRIS_SCORE, 31);
	 extractor = cv::DescriptorExtractor::create("ORB");
  }

void OrbFeaturesFinder::computeFeatureSet(const Mat& image, FeatureSet& features)const{
  moduleStarted("cpu features finder");
	double halfWidth = image.cols/2;
	double halfHeight = image.rows/2;
  for(int i = 0; i < sliceCount; i++){
	 vector<KeyPoint> tmp;
	 Rect roiRect(i*image.cols/sliceCount, 0, image.cols/sliceCount, image.rows);
	 Mat roi(image, roiRect);
	 orb->detect(roi, tmp);
	 if(i > 0){
		for(int i2 = tmp.size()-1; i2 >= 0; i2--){
		  tmp[i2].pt.x += i*image.cols/sliceCount;
		}
	 }
	 features.keyPoints.insert(features.keyPoints.end(), tmp.begin(), tmp.end());
#if DEBUG
	 cout << "size " << features.keyPoints.size() << endl;
#endif
  }
  extractor->compute(image, features.keyPoints, features.descriptors);
	for(int i = features.keyPoints.size()-1; i >= 0; i--){
		  features.keyPoints[i].pt.x -=  halfWidth;
			features.keyPoints[i].pt.y -=  halfHeight;
	}
  moduleEnded();
}
