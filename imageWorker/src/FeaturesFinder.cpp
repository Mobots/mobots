#include <opencv2/gpu/gpumat.hpp>
#include <iostream>

#include "FeaturesFinder.h"
#include "profile.h"

using namespace std;
using namespace cv::gpu;

/*void assertGpu(){
  int cudaCnt = cv::gpu::getCudaEnabledDeviceCount();
  if(cudaCnt < 1){
    stringstream ss;
    ss << "[GPU] cuda card count " << cudaCnt;
    cerr << ss.str() << endl;
    exit(1);
  }
}*/

void CpuFeaturesFinder::findFeatures(const cv::Mat& image, ImageFeatures& features)const{
  moduleStarted("cpu features finder");
  detector->detect(image, features.keyPoints);
  extractor->compute(image, features.keyPoints, features.descriptors);
  moduleEnded();
}

/*GpuFeaturesFinder::GpuFeaturesFinder(){
  assertGpu();
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





