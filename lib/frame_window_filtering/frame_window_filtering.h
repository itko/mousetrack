/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "generic/frame_window.h"

namespace MouseTrack {

class FrameWindowFiltering {
public:
  virtual ~FrameWindowFiltering() = default;
  virtual FrameWindow operator()(const FrameWindow &window) const = 0;
};

} // namespace MouseTrack
