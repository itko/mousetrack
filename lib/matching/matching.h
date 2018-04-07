/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "generic/cluster_chain.h"
#include "generic/cluster_descriptor.h"
#include <memory>
#include <vector>

namespace MouseTrack {

class Matching {
public:
  virtual ~Matching();

  /// Given a list of cluster chains, we find the best matching to append the
  /// given list of descriptors.
  ///
  /// descriptors: a list of descriptors from a new frame
  ///
  /// chains: a list of cluster chains to which we want to append one
  /// cluster from descriptors
  ///
  /// returns a vector of size descriptors.size(),
  /// which holds the index of the matching chain for each descriptor. If no
  /// match was found, the vector value is set to -1.
  ///
  /// Note: each cluster chain must have a descriptor for each frame of the
  /// chain
  virtual std::vector<long> operator()(
      const std::vector<std::shared_ptr<const ClusterDescriptor>> &descriptors,
      const std::vector<ClusterChain> &chains) = 0;
};

} // namespace MouseTrack
