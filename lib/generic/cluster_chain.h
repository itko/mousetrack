/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "cluster.h"
#include "cluster_descriptor.h"

#include <map>
#include <memory>


namespace MouseTrack {

/// Holds two maps of pointers to clusters and cluster descriptors
/// The indexes corresond to the according frame number
/// The key does not exist, if no cluster was found in a frame
class ClusterChain {
public:
    std::map<size_t, const Cluster*>& clusters();
    const std::map<size_t, const Cluster*>& clusters() const;
    std::map<size_t, std::shared_ptr<const ClusterDescriptor>>& descriptors();
    const std::map<size_t, std::shared_ptr<const ClusterDescriptor>>& descriptors() const;
private:
    std::map<size_t, std::shared_ptr<const ClusterDescriptor>> _descriptors;
    std::map<size_t, const Cluster*> _clusters;
};

} // MouseTrack

