/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "generic/point_cloud.h"

namespace MouseTrack {


/// General interface for point cloud processing.
///
class PointCloudFiltering {
public:
  virtual ~PointCloudFiltering() = default;

  /// Takes a point cloud and returns a new point cloud on which an operation was performed.
  virtual PointCloud operator()(const PointCloud &inCloud) const = 0;
};

} // namespace MouseTrack
