/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "generic/cluster_chain.h"
#include "generic/trajectory.h"

namespace MouseTrack {


class TrajectoryBuilder {
public:
    /// Creates a path in 3d space (trajectory) from a list of clusters and cluster descriptors (a cluster chain)
    virtual Trajectory operator()(const ClusterChain& chain) const = 0;
};


} // MouseTrack
