/// \file
/// Maintainer: Felice Serena
///

#include "disparity_registration_cpu_optimized.h"
#include <boost/log/trivial.hpp>
#include <omp.h>

namespace MouseTrack {

PointCloud DisparityRegistrationCpuOptimized::
operator()(const FrameWindow &window) const {
  const auto &frames = window.frames();

  // absolute transformation matrix relative to first camera
  auto Ts = absoluteTransformations(window);

  std::vector<Inverse> inverses(frames.size());
  for (size_t i = 0; i < frames.size(); i += 1) {
    Eigen::Matrix4d mat = frames[i].rotationCorrection * Ts[i];
    inverses[i] = prepareInverseTransformation(mat);
  }

  // store stuff in local variables
  const int border = frameBorder();
  const double xshift = correctingXShift();
  const double yshift = correctingYShift();
  const double minDisp = minDisparity() / 255.0;

  // flexibility to easily change row/column major
  typedef Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor> Mat;
  // name indices
  const int X = 0;
  const int Y = 1;
  const int D = 2;
  const int H = 3;

  // collect data in "frame" vectors
  std::vector<Mat> framePoints(frames.size());
  std::vector<std::vector<double>> frameIntensities(frames.size());

  // go through each frame, converting the disparity values to 3d points
  // relative to first camera
#pragma omp parallel for
  for (size_t i = 0; i < frames.size(); i += 1) {
    const auto &f = frames[i];
    const auto &disp = f.normalizedDisparityMap.zMap();
    int next_insert = 0;
    int expected = disp.size();
    // [x,y,disparity, 1]
    Mat pixels(4, expected);
    std::vector<double> intensities(expected);
    // convert each pixel
    for (int y = border - 1; y < disp.rows() - border; y += 1) {
      for (int x = border - 1; x < disp.cols() - border; x += 1) {
        double disparity = disp(y, x);
        if (disparity < minDisp) {
          // just skip those points
          continue;
        }
        pixels(X, next_insert) = x;
        pixels(Y, next_insert) = y;
        pixels(D, next_insert) = disparity;
        intensities[next_insert] = f.referencePicture(y, x);
        next_insert += 1;
      }
    }

    // shrink down/create view with correct size
    auto hom = pixels.block(0, 0, 4, next_insert);
    intensities.resize(next_insert);

    // disparity is returned between [0,1], but originally stored as [0,255]
    // 255*(f.baseline / disparity)
    hom.row(D) = (f.baseline / 255.0) / hom.row(D).array();

    // p.x() = (x + xshift - f.ccx) * (f.baseline * invDisparity);
    hom.row(X) = (hom.row(X).array() + (xshift - f.ccx)) * hom.row(D).array();

    // p.y() = (y + yshift - f.ccy) * (f.baseline * invDisparity);
    hom.row(Y) = (hom.row(Y).array() + (yshift - f.ccy)) * hom.row(D).array();

    // p.z() = f.focallength * (f.baseline * invDisparity);
    hom.row(D) = f.focallength * hom.row(D);

    // homogeneous 1
    hom.row(H).setConstant(1.0);

    // hom now holds homogeneous coordinates, find 3D points
    framePoints[i] = applyInverseTransformation(inverses[i], hom);
    frameIntensities[i] = intensities;
  }

  // merge into single point cloud
  int points_count = 0;
  for (auto &i : frameIntensities) {
    points_count += i.size();
  }
  PointCloud cloud;
  cloud.resize(points_count);
  // copy to cloud
  int next_insert = 0;
  for (size_t f = 0; f < frames.size(); ++f) {
    for (size_t i = 0; i < frameIntensities[f].size(); ++i) {
      auto p = cloud[next_insert++];
      p.x() = framePoints[f](X, i);
      p.y() = framePoints[f](Y, i);
      p.z() = framePoints[f](D, i);
      p.intensity(frameIntensities[f][i]);
    }
  }

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
