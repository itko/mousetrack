/// \file
/// Maintainer: Luzian Hug
///
///

#pragma once

#include "Eigen/Core"
#include "generic/cluster.h"
#include "generic/cluster_descriptor.h"
#include <memory>

namespace MouseTrack {

class TrajectoryBuilder {
public:
  /// Creates a new control point for a trajectory from a cluster and its
  /// descriptor
  virtual Eigen::Vector3d
  operator()(const std::shared_ptr<const ClusterDescriptor> descriptor,
             const Cluster &cluster, const PointCloud &cloud) const = 0;
};

} // namespace MouseTrack
