/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "trajectory_builder.h"

namespace MouseTrack {




    /// Creates a new control point for a trajectory from a cluster and its descriptor
     Eigen::Vector3d TrajectoryBuilder::operator()(const std::shared_ptr<const ClusterDescriptor> descriptor, const Cluster& cluster, const PointCloud& cloud) const {
       Eigen::VectorXd cog4 = cluster.center_of_gravity(cloud);
       return Eigen::Vector3d(cog4[0],cog4[1],cog4[2]);
     }



} // MouseTrack
