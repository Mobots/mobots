#include <opencv2/gpu/gpumat.hpp>

#include "FeaturesFinder.h"

using namespace cv::gpu;

void assertGpu(){
  int cudaCnt = cv::gpu::getCudaEnabledDeviceCount();
  if(cudaCnt < 1){
    stringstream ss;
    ss << "[GPU] cuda card count " << cudaCnt;
    throw ss.str();
  }
}

void CpuFeaturesFinder::findFeatures(const cv::Mat& image, ImageFeatures& features)const{
  detector->detect(image, features.keypoints);
  extractor->compute(image, features.keypoints, features.descriptors);
}

GpuFeaturesFinder::GpuFeaturesFinder(){
  assertGpu();
}

void SurfGpuFeaturesFinder::findFeatures(const cv::Mat& image, ImageFeatures& features)const{
  //requires grayscale
  
  /*GpuMat keypoints;
  GpuMat descriptors;
  detector(image, GpuMat(), keypoints);
  detector(image, GpuMat(), keypoints, descriptors, true); //true?
  detector.downloadKeypoints(keypoints, features.keypoints);
  descriptors.download(features.descriptors);*/
}




