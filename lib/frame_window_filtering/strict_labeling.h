/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "frame_window_filtering.h"

#include <memory>

namespace MouseTrack {

/// Assumes existing labels
/// For each pixel, it sets one label to 1 and all others to 0
class StrictLabeling : public FrameWindowFiltering {
public:
  virtual FrameWindow operator()(const FrameWindow &window) const;
};

} // namespace MouseTrack
