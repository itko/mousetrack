/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "registration.h"

namespace MouseTrack {

class DisparityRegistration : public Registration {
public:
  PointCloud operator()(const FrameWindow &window) const;

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
