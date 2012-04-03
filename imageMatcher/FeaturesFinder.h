#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>

#pragma once

using namespace std;
using namespace cv;

typedef struct{
  std::vector<KeyPoint> keypoints;
  cv::Mat descriptors;
} ImageFeatures;

class FeaturesFinder{
public:
  virtual ~FeaturesFinder(){}
  virtual void findFeatures(const Mat &image, ImageFeatures& features) const = 0;
};

class CpuFeaturesFinder : public FeaturesFinder{
private:
  Ptr<FeatureDetector> detector;
  Ptr<DescriptorExtractor> extractor;
public:
  void findFeatures(const Mat& image, ImageFeatures& features) const;
  void set(Ptr<FeatureDetector> detector, Ptr<DescriptorExtractor> extractor){
    this->detector = detector;
    this->extractor = extractor;
  }
  virtual ~CpuFeaturesFinder(){}
protected:
  CpuFeaturesFinder(){}
};

class SurfFeaturesFinder : public CpuFeaturesFinder{
public:
  /**
   * first half of the arguments are for detector, second half for extractor
   */
  SurfFeaturesFinder(double hessianThreshold = 400, int octaves = 3, int octaveLayers = 4,
		     int nOctaves = 4, int nOctaveLayers = 2, bool extended = false){
    set(new SurfFeatureDetector(hessianThreshold, octaves, octaveLayers), 
      new SurfDescriptorExtractor(nOctaves, nOctaveLayers, extended));
  }
  ~SurfFeaturesFinder(){}
};

class OrbFeaturesFinder : public CpuFeaturesFinder{
public:
  OrbFeaturesFinder(size_t n_features = 700, ORB::CommonParams params = ORB::CommonParams()){
    set(new OrbFeatureDetector(n_features, params), new OrbDescriptorExtractor(params));    
  }
  ~OrbFeaturesFinder(){} 
};

class GpuFeaturesFinder: public FeaturesFinder{
public:
  GpuFeaturesFinder();
  virtual void findFeatures(const Mat& image, ImageFeatures& features) const = 0;
  virtual ~GpuFeaturesFinder(){}
};

class SurfGpuFeaturesFinder : public GpuFeaturesFinder{
private:
  gpu::SURF_GPU detector; 
public:
  SurfGpuFeaturesFinder(){}
  void findFeatures(const Mat& image, ImageFeatures& features) const;
  ~SurfGpuFeaturesFinder(){}
};
