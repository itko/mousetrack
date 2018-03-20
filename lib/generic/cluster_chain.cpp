/// \file
/// Maintainer: Felice Serena
///
///

#include "cluster_chain.h"



namespace MouseTrack {

std::map<size_t, const Cluster*>& ClusterChain::clusters() {
    return _clusters;
}

const std::map<size_t, const Cluster*>& ClusterChain::clusters() const {
    return _clusters;
}

std::map<size_t, std::shared_ptr<const ClusterDescriptor>>& ClusterChain::descriptors() {
    return _descriptors;
}

const std::map<size_t, std::shared_ptr<const ClusterDescriptor>>& ClusterChain::descriptors() const {
    return _descriptors;
}

} // MouseTrack
