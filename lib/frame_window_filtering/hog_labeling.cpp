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

FrameWindow HogLabeling::operator()(const FrameWindow &window) const {
  FrameWindow result = window;
  for (size_t f = 0; f < result.frames().size(); ++f) {
    Frame &frame = result.frames()[f];
    cv::Mat img;
    PictureI eig = (frame.referencePicture * 255.0).cast<PictureI::Scalar>();
    cv::eigen2cv(eig, img);
    std::vector<float> descriptors;
    cv::HOGDescriptor hog;
    hog.compute(img, descriptors);
    BOOST_LOG_TRIVIAL(trace)
        << "Found " << descriptors.size() << " HOG descriptors in frame " << f;
  }
  return result;
}

} // namespace MouseTrack
