#include <opencv2/gpu/gpumat.hpp>
#include <iostream>

#include "feature_detector/FeaturesFinder.h"
#include "profile.h"

using namespace std;
using namespace cv;

/*void assertGpu(){
  int cudaCnt = cv::gpu::getCudaEnabledDeviceCount();
  if(cudaCnt < 1){
    stringstream ss;
    ss << "[GPU] cuda card count " << cudaCnt;
    cerr << ss.str() << endl;
    exit(1);
  }
}*/

Ptr<FeaturesFinder> FeaturesFinder::getDefault(){
  return new OrbFeaturesFinder;
}

void SurfFeaturesFinder::computeFeatureSet(const Mat& image, FeatureSet& features)const{
  moduleStarted("cpu features finder");
  (*surf)(image, Mat(), features.keyPoints, features.descriptors, true);
  moduleEnded();
}

void OrbFeaturesFinder::computeFeatureSet(const Mat& image, FeatureSet& features)const{
  moduleStarted("cpu features finder");
  (*orb)(image, Mat(), features.keyPoints, features.descriptors, false);
  moduleEnded();
}

void FastFeaturesFinder::computeFeatureSet(const Mat& image, FeatureSet& features)const{
  moduleStarted("cpu features finder");
  FAST(image, features.keyPoints, threshold, nonmaxSuppression);
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





