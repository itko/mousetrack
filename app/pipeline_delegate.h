/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "generic/types.h"

namespace MouseTrack {

/// Implement this interface if you want to control the pipeline
/// at run time in an interactive manner.
class PipelineDelegate {
public:
  virtual ~PipelineDelegate() = default;

  /// `true` if there exists a frame to process.
  /// `false` if no more frames are available.
  virtual bool hasNextFrame() const = 0;

  /// Returns the frame number that should be processed
  /// in the next run of the pipeline
  virtual FrameNumber nextFrame() = 0;
};
} // namespace MouseTrack
