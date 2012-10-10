#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace drawing{
  
  void drawMatches( const cv::Mat& img1, const std::vector<cv::KeyPoint>& keypoints1,
                  const cv::Mat& img2, const std::vector<cv::KeyPoint>& keypoints2,
                  const std::vector<cv::DMatch>& matches1to2, cv::Mat& outImg,
                  const cv::Scalar& matchColor, const cv::Scalar& singlePointColor,
                  const std::vector<char>& matchesMask, int flags, int maxCount);
  void drawMatches2(const cv::Mat& img1, const std::vector<cv::Point2f>& keypoints1,
                  const cv::Mat& img2, const std::vector<cv::Point2f>& keypoints2,
                  cv::Mat& outImg,
                  const cv::Scalar& matchColor, const cv::Scalar& singlePointColor,
						int flags, int maxCount);
}