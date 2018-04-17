/// \file
/// Maintainer: Felice Serena
///
///

#include "disparity_gaussian_blur.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace MouseTrack {

FrameWindow DisparityGaussianBlur::operator()(const FrameWindow &window) const {
  FrameWindow result = window;
  for (size_t i = 0; i < window.frames().size(); ++i) {
    const Frame &f = window.frames()[i];
    auto &disp = f.normalizedDisparityMap.zMap();
    cv::Mat raw, smooth;
    int kSize = 7;
    cv::eigen2cv(disp, raw);
    cv::GaussianBlur(raw, smooth, cv::Size(kSize, kSize), 0, 0);
    cv::cv2eigen(smooth, result.frames()[i].normalizedDisparityMap.zMap());
  }
  return result;
}

} // namespace MouseTrack
