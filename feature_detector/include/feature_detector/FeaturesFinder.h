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

class SurfFeaturesFinder : public CpuFeaturesFinder{
public:
  cv::Ptr<cv::SURF> surf;
  /**
    hessianThreshold – Threshold for hessian keypoint detector used in SURF.
    nOctaves – Number of pyramid octaves the keypoint detector will use.
    nOctaveLayers – Number of octave layers within each octave.
    extended – Extended descriptor flag (true - use extended 128-element descriptors; false - use 64-element descriptors).
    upright – Up-right or rotated features flag (true - do not compute orientation of features; false - compute orientation).
   */
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
   */
  FastFeaturesFinder(int threshold, bool nonmaxSuppression, int type)
  : threshold(threshold), nonmaxSuppression(nonmaxSuppression), type(type){}
  virtual void computeFeatureSet(const cv::Mat& image, FeatureSet& features) const;
  ~FastFeaturesFinder(){}
};


class OrbFeaturesFinder : public CpuFeaturesFinder{
  cv::Ptr<cv::ORB> orb;
public:
  /**
   * 
    nfeatures – The maximum number of features to retain.
    scaleFactor – Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.
    nlevels – The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels).
    edgeThreshold – This is size of the border where the features are not detected. It should roughly match the patchSize parameter.
    firstLevel – It should be 0 in the current implementation.
    WTA_K – The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).
    scoreType – The default HARRIS_SCORE means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.
    patchSize – size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.
  */
  OrbFeaturesFinder(int nfeatures=500, float scaleFactor=1.2f, int nlevels=8, int edgeThreshold=31, 
		    int firstLevel=0, int WTA_K=2, int scoreType=cv::ORB::HARRIS_SCORE, int patchSize=31){
    orb = new cv::ORB(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize);
  }
  virtual void computeFeatureSet(const cv::Mat& image, FeatureSet& features) const;
  ~OrbFeaturesFinder(){} 
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
