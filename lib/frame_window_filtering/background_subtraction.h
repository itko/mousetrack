/// \file
/// Maintainer: Luzian Hug
///

#pragma once

#include "frame_window_filtering.h"
#include <string>

namespace MouseTrack {

/// This subtracts pictures of a given frame (cage_frame) from the target frame
/// (cage with mouse).
///
class BackgroundSubtraction : public FrameWindowFiltering {
public:
  BackgroundSubtraction();
  virtual FrameWindow operator()(const FrameWindow &window) const;

  const FrameWindow &cage_frame() const;
  void cage_frame(FrameWindow &cage_frame);

  double otsu_factor() const;
  void otsu_factor(double otsu_factor);

private:
  FrameWindow _cage_frame = FrameWindow();
  double _otsu_factor = 1;
};

} // namespace MouseTrack
