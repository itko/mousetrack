/// \file
/// Maintainer: Felice Serena
///
///

#include "write_png.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <boost/log/trivial.hpp>

namespace MouseTrack {

bool write_png(const PictureD &img, const std::string &path) {
  PictureI im = (255 * img).cast<PictureI::Scalar>();
  return write_png(im, path);
}

bool write_png(const PictureI &img, const std::string &path) {
  cv::Mat im;
  cv::eigen2cv(img, im);
  try {
    return cv::imwrite(path, im);
  } catch (std::runtime_error &ex) {
    BOOST_LOG_TRIVIAL(warning)
        << "Exception converting image to PNG format: " << ex.what();
    return false;
  }
}

} // namespace MouseTrack
