/// \file
/// Maintainer: Felice Serena
///

#include "disparity_registration.h"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace MouseTrack {

std::vector<Eigen::Matrix4d> DisparityRegistration::absoluteTransformations(
    const FrameWindow &window) const {
  const auto &frames = window.frames();
  std::vector<Eigen::Matrix4d> Ts(frames.size());
  Ts[0] = Eigen::Matrix4d::Identity();
  for (size_t i = 1; i < frames.size(); i += 1) {
    Ts[i] =
        frames[i].camChainPicture * frames[i - 1].camChainDisparity * Ts[i - 1];
  }
  return Ts;
}

DisparityRegistration::Inverse
DisparityRegistration::prepareInverseTransformation(
    const Eigen::Matrix4d &mat) const {
  // For the moment, we just return the inverse.
  // We could also use a decomposition object from Eigen
  // to increase robustness.
  // Or we could use domain knowledge about the
  // transformation to speed up computations.
  return mat.inverse();
}

Eigen::Vector4d DisparityRegistration::applyInverseTransformation(
    const DisparityRegistration::Inverse &inv, const Eigen::Vector4d &p) const {
  // for the moment just a simple multiplication
  return inv * p;
}

double &DisparityRegistration::minDisparity() { return _min_disparity; }

/// Lowest disparity value we accept (we remove points at infinity)
const double &DisparityRegistration::minDisparity() const {
  return _min_disparity;
}

/// Set X shift to correct disparity map position
int &DisparityRegistration::correctingXShift() { return _xshift; }

/// Read X shift to correct disparity map position
const int &DisparityRegistration::correctingXShift() const { return _xshift; }

/// Set Y shift to correct disparity map position
int &DisparityRegistration::correctingYShift() { return _yshift; }

/// Read Y shift to correct disparity map position
const int &DisparityRegistration::correctingYShift() const { return _yshift; }

/// Ignores boder of n pixels around disparity map
int &DisparityRegistration::frameBoder() { return _frame_border; }

/// Ignores boder of n pixels around disparity map
const int &DisparityRegistration::frameBorder() const { return _frame_border; }

} // namespace MouseTrack
