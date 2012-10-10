#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/nonfree/features2d.hpp>

#pragma once

typedef struct{
  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;
} FeatureSet;


class FeaturesFinder{
public:
  virtual ~FeaturesFinder(){}
  /**
	* compute FeatureSet for a given image
	*/
  virtual void computeFeatureSet(const cv::Mat& image, FeatureSet& features) const = 0;
  static cv::Ptr<FeaturesFinder> getDefault();
};

class CpuFeaturesFinder : public FeaturesFinder{
private:
public:
  virtual void computeFeatureSet(const cv::Mat& image, FeatureSet& features) const = 0;
  virtual ~CpuFeaturesFinder(){}
protected:
  CpuFeaturesFinder(){}
};

/**
 * ORB is currently the used algorithm for feature detection
 */
class OrbFeaturesFinder : public CpuFeaturesFinder{
  cv::Ptr<cv::ORB> orb;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  int sliceCount;
public:
  /**
    int nfeatures – The maximum number of features to retain.
    int sliceCount - the amount of slices you want your pizza sliced into
  */
  OrbFeaturesFinder(int nfeatures=300, int sliceCount = 5);
  virtual void computeFeatureSet(const cv::Mat& image, FeatureSet& features) const;
  ~OrbFeaturesFinder(){} 
};

/*
class SurfFeaturesFinder : public CpuFeaturesFinder{
public:
  cv::Ptr<cv::SURF> surf;
  /**
    hessianThreshold – Threshold for hessian keypoint detector used in SURF.
    nOctaves – Number of pyramid octaves the keypoint detector will use.
    nOctaveLayers – Number of octave layers within each octave.
    extended – Extended descriptor flag (true - use extended 128-element descriptors; false - use 64-element descriptors).
    upright – Up-right or rotated features flag (true - do not compute orientation of features; false - compute orientation).
   *//*
  SurfFeaturesFinder(double hessianThreshold = 400, int nOctaves = 4, int nOctaveLayers = 2,
		     bool extended = false, bool upright = false){
    surf = new cv::SURF(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);
  }
  virtual void computeFeatureSet(const cv::Mat& image, FeatureSet& features) const;
  ~SurfFeaturesFinder(){}
};

class FastFeaturesFinder : public CpuFeaturesFinder{
  int threshold;
  bool nonmaxSuppression;
  int type;
public:
  /**
   * threshold – threshold on difference between intensity of the central pixel and pixels of a circle around this pixel.
     nonmaxSupression – if true, non-maximum suppression is applied to detected corners (keypoints).
     type – one of the three neighborhoods as defined in the paper: FastFeatureDetector::TYPE_9_16, FastFeatureDetector::TYPE_7_12, FastFeatureDetector::TYPE_5_8
   *//*
  FastFeaturesFinder(int threshold, bool nonmaxSuppression, int type)
  : threshold(threshold), nonmaxSuppression(nonmaxSuppression), type(type){}
  virtual void computeFeatureSet(const cv::Mat& image, FeatureSet& features) const;
  ~FastFeaturesFinder(){}
};


/*class GpuFeaturesFinder: public FeaturesFinder{
public:
  GpuFeaturesFinder();
  virtual void findFeatures(const cv::Mat& image, FeatureSet& features) const = 0;
  virtual ~GpuFeaturesFinder(){}
};

class SurfGpuFeaturesFinder : public GpuFeaturesFinder{
private:
  gpu::SURF_GPU detector; 
public:
  SurfGpuFeaturesFinder(){}
  void findFeatures(const Mat& image, ImageFeatures& features) const;
  ~SurfGpuFeaturesFinder(){}
};*/