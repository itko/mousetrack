/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "generic/disparity_map.h"
#include "generic/frame_window.h"
#include "generic/types.h"

namespace MouseTrack {

class Reader {
public:
  /// true if the reader is ready to provide output
  virtual bool valid() const = 0;

  /// index of first existing frame
  virtual FrameNumber beginFrame() const = 0;

  /// Index of first non existing frame at the end
  virtual FrameNumber endFrame() const = 0;

  /// Creates the Frame Window for frame index f, might read from disk/network
  /// This method is allowed to change the result of `endFrame`, if new frames
  /// get available. It might also block and wait for the next frame from the
  /// network. This is implementation dependent.
  virtual FrameWindow operator()(FrameNumber f) = 0;
};

} // namespace MouseTrack
