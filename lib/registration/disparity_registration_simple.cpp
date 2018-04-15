/// \file
/// Maintainer: Felice Serena
///

#include "disparity_registration_simple.h"
#include <boost/log/trivial.hpp>

namespace MouseTrack {

PointCloud DisparityRegistrationSimple::
operator()(const FrameWindow &window) const {
  const auto &frames = window.frames();

  // how many points will there be at most?
  int expected_points = 0;
  for (const auto &f : frames) {
    expected_points += f.normalizedDisparityMap.zMap().size();
  }

  // absolute transformation matrix relative to first camera
  auto Ts = absoluteTransformations(window);

  std::vector<Inverse> inverses(frames.size());
  for (size_t i = 0; i < frames.size(); i += 1) {
    Eigen::Matrix4d mat = frames[i].rotationCorrection * Ts[i];
    inverses[i] = prepareInverseTransformation(mat);
  }

  // Allocate point cloud enough large to capture all points
  PointCloud cloud;
  cloud.resize(expected_points);
  int next_insert = 0;
  const int border = frameBorder();
  const double xshift = correctingXShift();
  const double yshift = correctingYShift();
  const double minDisp = minDisparity();
  // go through each frame, converting the disparity values to 3d points
  // relative to first camera
  for (size_t i = 0; i < frames.size(); i += 1) {
    const auto &f = frames[i];
    const auto &disp = f.normalizedDisparityMap.zMap();
    // convert each pixel
    for (int y = border - 1; y < disp.rows() - border; y += 1) {
      for (int x = border - 1; x < disp.cols() - border; x += 1) {
        double disparity = 255 * disp(y, x); // disparity is returned between
                                             // [0,1], but originally stored as
                                             // [0,255]
        if (disparity < minDisp) {
          // just skip those points
          continue;
        }
        const double invDisparity = 1.0 / disparity;
        auto p = cloud[next_insert];
        p.x() = (x + xshift - f.ccx) * f.baseline * invDisparity;
        p.y() = (y + yshift - f.ccy) * f.baseline * invDisparity;
        p.z() = f.focallength * f.baseline * invDisparity;

        Eigen::Vector4d tmp = applyInverseTransformation(
            inverses[i], Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0));
        p.x() = tmp[0];
        p.y() = tmp[1];
        p.z() = tmp[2];
        p.intensity(f.referencePicture(y, x));
        next_insert += 1;
      }
    }
  }
  cloud.resize(next_insert); // shrink to actual number of points

  auto min = cloud.min();
  auto max = cloud.max();

  BOOST_LOG_TRIVIAL(debug) << "Found point cloud with " << cloud.size()
                           << " points, xyz-min: [" << min[0] << ", " << min[1]
                           << ", " << min[2] << "], xyz-max: [" << max[0]
                           << ", " << max[1] << ", " << max[2] << "]"
                           << std::flush;
  return cloud;
}

} // namespace MouseTrack
