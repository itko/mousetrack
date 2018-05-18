/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "frame_window_filtering.h"

namespace MouseTrack {

class HogLabeling : public FrameWindowFiltering {
public:
  virtual FrameWindow operator()(const FrameWindow &window) const;
};

} // namespace MouseTrack
