#include <opencv2/gpu/gpumat.hpp>
#include <iostream>

#include "FeaturesFinder.h"
#include "profile.h"

using namespace std;
using namespace cv::gpu;

void assertGpu(){
  int cudaCnt = cv::gpu::getCudaEnabledDeviceCount();
  if(cudaCnt < 1){
    stringstream ss;
    ss << "[GPU] cuda card count " << cudaCnt;
    cerr << ss.str() << endl;
    exit(1);
  }
}

void CpuFeaturesFinder::findFeatures(const cv::Mat& image, ImageFeatures& features)const{
  moduleStarted("cpu features finder");
  detector->detect(image, features.keypoints);
  extractor->compute(image, features.keypoints, features.descriptors);
  moduleEnded();
}

GpuFeaturesFinder::GpuFeaturesFinder(){
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
}




