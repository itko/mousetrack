/// \file
/// Maintainer: Luzian Hug
///
///

#pragma once

#include "Eigen/Core"
#include "generic/cluster.h"
#include "generic/cluster_descriptor.h"
#include "trajectory_builder.h"
#include <memory>

namespace MouseTrack {

class CogTrajectoryBuilder : public TrajectoryBuilder {
public:
  /// Creates a new control point for a trajectory from a cluster and its
  /// descriptor
  Eigen::Vector3d
  operator()(const std::shared_ptr<const ClusterDescriptor> descriptor,
             const Cluster &cluster, const PointCloud &cloud) const;
};

} // namespace MouseTrack
