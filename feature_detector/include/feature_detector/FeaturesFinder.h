#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>

#pragma once

typedef struct{
  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;
} ImageFeatures;

class FeaturesFinder{
public:
  virtual ~FeaturesFinder(){}
  virtual void findFeatures(const cv::Mat &image, ImageFeatures& features) const = 0;
};

class CpuFeaturesFinder : public FeaturesFinder{
private:
 cv::Ptr<cv::FeatureDetector> detector;
 cv::Ptr<cv::DescriptorExtractor> extractor;
public:
  void findFeatures(const cv::Mat& image, ImageFeatures& features) const;
  void set(cv::Ptr<cv::FeatureDetector> detector, cv::Ptr<cv::DescriptorExtractor> extractor){
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
    set(new cv::SurfFeatureDetector(hessianThreshold, octaves, octaveLayers), 
      new cv::SurfDescriptorExtractor(nOctaves, nOctaveLayers, extended));
  }
  ~SurfFeaturesFinder(){}
};

class FastFeaturesFinder : public CpuFeaturesFinder{
public:
  /**
   * first half of the arguments are for detector, second half for extractor
   */
  FastFeaturesFinder(int threshold=1, bool nonmaxSuppression=true,
		     int nOctaves = 4, int nOctaveLayers = 2, bool extended = false){
    set(new cv::FastFeatureDetector(threshold, nonmaxSuppression), 
      new cv::SurfDescriptorExtractor(nOctaves, nOctaveLayers, extended));
  }
  ~FastFeaturesFinder(){}
};


class OrbFeaturesFinder : public CpuFeaturesFinder{
public:
  OrbFeaturesFinder(size_t n_features = 700, cv::ORB::CommonParams params = cv::ORB::CommonParams()){
    set(new cv::OrbFeatureDetector(n_features, params), new cv::OrbDescriptorExtractor(params));    
  }
  ~OrbFeaturesFinder(){} 
};

class GpuFeaturesFinder: public FeaturesFinder{
public:
  GpuFeaturesFinder();
  virtual void findFeatures(const cv::Mat& image, ImageFeatures& features) const = 0;
  virtual ~GpuFeaturesFinder(){}
};

/*class SurfGpuFeaturesFinder : public GpuFeaturesFinder{
private:
  gpu::SURF_GPU detector; 
public:
  SurfGpuFeaturesFinder(){}
  void findFeatures(const Mat& image, ImageFeatures& features) const;
  ~SurfGpuFeaturesFinder(){}
};*/
