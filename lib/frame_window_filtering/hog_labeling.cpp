/// \file
/// Maintainer: Felice Serena
///
///

#include "hog_labeling.h"

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <boost/log/trivial.hpp>

namespace MouseTrack {

/// Returns a n x 2 matrix of (x,y) pairs of valid corner coordinates for
/// sliding windows of the desired size such that they are within the given
/// image dimensions.
///
std::vector<cv::Point> slidingWindows(int imgWidth, int imgHeight, int stepSize,
                                      int windowWidth, int windowHeight) {
  std::vector<cv::Point> result;
  for (int x = 0; x < imgWidth - windowWidth; x += stepSize) {
    for (int y = 0; y < imgHeight - windowHeight; y += stepSize) {
      result.push_back(cv::Point(x, y));
    }
  }
  return result;
} // namespace MouseTrack

FrameWindow HogLabeling::operator()(const FrameWindow &window) const {
  if (window.frames().empty()) {
    return window;
  }
  FrameWindow result = window;
  int windowWidth = 64;
  int windowHeight = 64;
  int stepSize = 64 / 4;

  // hog settings
  cv::Size windowSize(windowWidth, windowHeight);
  cv::Size blockSize(16, 16);
  cv::Size blockStride(8, 8);
  cv::Size cellSize(8, 8);
  int nbins = 9;

  // create hog descriptor: it is able to tansform images patches to feature
  // vectors
  cv::HOGDescriptor hog(windowSize, blockSize, blockStride, cellSize, nbins);

  // compute settings
  cv::Size windowStride;
  cv::Size padding;

  // build locations for sliding window
  const auto &first = result.frames()[0];
  std::vector<cv::Point> locations = slidingWindows(
      first.referencePicture.cols(), first.referencePicture.rows(), stepSize,
      windowWidth, windowHeight);

  BOOST_LOG_TRIVIAL(trace) << "Created " << locations.size()
                           << " sliding window locations to check.";

  for (size_t f = 0; f < result.frames().size(); ++f) {
    Frame &frame = result.frames()[f];
    cv::Mat img;
    PictureI eig = (frame.referencePicture * 255.0).cast<PictureI::Scalar>();
    cv::eigen2cv(eig, img);

    // holds `locations.size()` descriptors of size hog.getDescriptorSize()
    std::vector<float> descriptors;
    hog.compute(img, descriptors, windowStride, padding, locations);
    BOOST_LOG_TRIVIAL(trace)
        << "Found " << (descriptors.size() / hog.getDescriptorSize())
        << " HOG descriptors of size " << hog.getDescriptorSize() << " at "
        << locations.size() << " locations in frame " << f;
  }
  return result;
}

} // namespace MouseTrack
