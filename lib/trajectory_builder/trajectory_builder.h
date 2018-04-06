/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "generic/cluster.h"
#include "generic/cluster_descriptor.h"
#include "Eigen/Core"
#include <memory>

namespace MouseTrack {


class TrajectoryBuilder {
public:
    /// Creates a new control point for a trajectory from a cluster and its descriptor
    virtual Eigen::Vector3d operator()(const std::shared_ptr<const ClusterDescriptor> descriptor, const Cluster& cluster, const PointCloud& cloud) const = 0;
};


} // MouseTrack
