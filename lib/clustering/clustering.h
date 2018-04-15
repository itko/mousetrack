/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "generic/cluster.h"
#include "generic/point_cloud.h"
#include <vector>

namespace MouseTrack {

/// Interface for Clustering algorithms
class Clustering {
public:
  /// Takes a point cloud and splits it into clusters.
  virtual std::vector<Cluster> operator()(const PointCloud &cloud) const = 0;
};

} // namespace MouseTrack
