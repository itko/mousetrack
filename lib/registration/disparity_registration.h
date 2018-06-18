/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "registration.h"
#include <Eigen/Core>
#include <vector>

namespace MouseTrack {

/// Registration algorithm for known disparity maps.
class DisparityRegistration : public Registration {
public:
  /// Simple, sequential, straight forward solution.
  /// Think of it as the most basic implementation suitable as reference.
  ///
  /// Overwrite this method for an improved implementation
  virtual PointCloud operator()(const FrameWindow &window) const;

  /// Lowest disparity value we accept (we remove points at infinity)
  double &minDisparity();

  /// Lowest disparity value we accept (we remove points at infinity)
  const double &minDisparity() const;

  /// Set X shift to correct disparity map position
  int &correctingXShift();

  /// Read X shift to correct disparity map position
  const int &correctingXShift() const;

  /// Set Y shift to correct disparity map position
  int &correctingYShift();

  /// Read Y shift to correct disparity map position
  const int &correctingYShift() const;

  /// Ignores boder of n pixels around disparity map
  int &frameBoder();

  /// Ignores boder of n pixels around disparity map
  const int &frameBorder() const;

protected:
  /// Typedef for Inverse concept: There might be reasons
  /// where we would like not to compute the inverse, but some
  /// kind of decomposition because it's more robust or so.
  /// This gives us the necessary flexibility
  typedef Eigen::Matrix4d Inverse;

  /// Performs calculations to easily apply the inverse of `mat` to a 4d
  /// homogeneous point.
  Inverse prepareInverseTransformation(const Eigen::Matrix4d &mat) const;

  /// Apply the inverse transformation object
  Eigen::Vector4d applyInverseTransformation(const Inverse &inv,
                                             const Eigen::Vector4d &p) const;

  /// Apply inverse transformation object to matrix of points
  ///
  /// `ps`: 4 x #P matrix
  template <typename Derived>
  Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>
  applyInverseTransformation(const Inverse &inv,
                             const Eigen::MatrixBase<Derived> &ps) const {
    return inv * ps;
  }

  /// Convenience method to calculate the aboslute transformations for all
  /// cameras. absolute transformation matrix relative to first camera
  std::vector<Eigen::Matrix4d>
  absoluteTransformations(const FrameWindow &window) const;

private:
  /// constant from fpga set up
  double _min_disparity = 1 * 2 + 32;
  /// constant from fpga set up
  int _xshift = 22;
  /// constant from fpga set up
  int _yshift = -8;
  /// constant from fpga set up
  int _frame_border = 80;
};

} // namespace MouseTrack
