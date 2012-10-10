#include <opencv2/gpu/gpumat.hpp>
#include <iostream>

#include "feature_detector/FeaturesFinder.h"
#include "profile.h"

using namespace std;
using namespace cv;

Ptr<FeaturesFinder> FeaturesFinder::getDefault(){
  return new OrbFeaturesFinder(800, 1);
  //return new SurfFeaturesFinder;
}

OrbFeaturesFinder::OrbFeaturesFinder(int nfeatures, int sliceCount)
  :sliceCount(sliceCount){
    orb = new cv::ORB(nfeatures);
	 extractor = cv::DescriptorExtractor::create("ORB");
  }

void OrbFeaturesFinder::computeFeatureSet(const Mat& image, FeatureSet& features)const{
  moduleStarted("cpu features finder");
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
	 cout << "size " << features.keyPoints.size() << endl;
  }
  extractor->compute(image, features.keyPoints, features.descriptors);

  moduleEnded();
}


/*
 * == unused classes: GPU methods, SURF, FAST, etc. ==
 */
/*GpuFeaturesFinder::GpuFeaturesFinder(){
  assertGpu();
}

void SurfFeaturesFinder::computeFeatureSet(const Mat& image, FeatureSet& features)const{
  moduleStarted("cpu features finder");
  surf->operator()(image, Mat(), features.keyPoints, features.descriptors, false);
  moduleEnded();
}

void FastFeaturesFinder::computeFeatureSet(const Mat& image, FeatureSet& features)const{
  moduleStarted("cpu features finder");
  FAST(image, features.keyPoints, threshold, nonmaxSuppression);
  moduleEnded();
}

void SurfGpuFeaturesFinder::findFeatures(const cv::Mat& image, ImageFeatures& features)const{
  //requires grayscale
  SURF_GPU detector(600); 
  GpuMat gpuImage;
  gpuImage.upload(image);
  GpuMat keypoints;
  GpuMat descriptors;
  detector(gpuImage, GpuMat(), keypoints, descriptors); //true?
  detector.downloadKeypoints(keypoints, features.keypoints);
  descriptors.download(features.descriptors);
}*/

/*~CpuFeaturesFinder(){}
~SurfFeaturesFinder(){}
~OrbFeaturesFinder(){}

OrbFeaturesFinder(size_t n_features = 700, ORB::CommonParams params = ORB::CommonParams()){
    set(new OrbFeatureDetector(n_features, params), new OrbDescriptorExtractor(params));    
}
FastFeaturesFinder(int threshold=1, bool nonmaxSuppression=true,
		     int nOctaves = 4, int nOctaveLayers = 2, bool extended = false){
    set(new FastFeatureDetector(threshold, nonmaxSuppression), 
      new SurfDescriptorExtractor(nOctaves, nOctaveLayers, extended));
}
SurfFeaturesFinder(double hessianThreshold = 400, int octaves = 3, int octaveLayers = 4,
		     int nOctaves = 4, int nOctaveLayers = 2, bool extended = false){
    set(new SurfFeatureDetector(hessianThreshold, octaves, octaveLayers), 
      new SurfDescriptorExtractor(nOctaves, nOctaveLayers, extended));
}*/

/*void assertGpu(){
  int cudaCnt = cv::gpu::getCudaEnabledDeviceCount();
  if(cudaCnt < 1){
    stringstream ss;
    ss << "[GPU] cuda card count " << cudaCnt;
    cerr << ss.str() << endl;
    exit(1);
  }
}*/