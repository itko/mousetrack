/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "generic/frame_window.h"
#include "generic/point_cloud.h"

namespace MouseTrack {

/// General Registration interface: A set of 2D images is given from which a 3D point cloud has to be created.
class Registration {
public:
  /// Assumes the camchain matrices to be chained according to the indices of
  /// the vector
  virtual PointCloud operator()(const FrameWindow &window) const = 0;
};

} // namespace MouseTrack
