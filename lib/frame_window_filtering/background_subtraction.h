/// \file
/// Maintainer: Luzian Hug
///

#pragma once

#include "frame_window_filtering.h"

namespace MouseTrack {


/// This subtracts pictures of a given frame (cage_frame) from the target frame (cage with mouse).
/// 
class BackgroundSubtraction : public FrameWindowFiltering {
public:
  BackgroundSubtraction(const FrameWindow* const cage_frame);
  FrameWindow operator()(const FrameWindow &window);

  const FrameWindow *cage_frame() const;
  void cage_frame(FrameWindow *cage_frame);

  const double threshold() const;
  void threshold(double threshold);

private:
  const FrameWindow* _cage_frame;
  double _threshold = 0.01;
};

} // namespace MouseTrack
