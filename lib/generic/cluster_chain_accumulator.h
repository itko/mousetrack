/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "cluster_chain.h"
#include "types.h"

namespace MouseTrack {

/// Appends `clusters` and `descriptors` to `chains` according to `matches` and
/// `frame`
void addToClusterChain(
    std::vector<ClusterChain> &chains, const std::vector<long> &matches,
    FrameNumber frame, const std::vector<Cluster> &clusters,
    const std::vector<std::shared_ptr<const ClusterDescriptor>> &descriptors);

} // MouseTrack
