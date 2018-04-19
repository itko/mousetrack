/// \file
/// Maintainer: Felice Serena
///
///

#include "cluster_chain_accumulator.h"

namespace MouseTrack {

void addToClusterChain(
    std::vector<ClusterChain> &chains, const std::vector<long> &matches,
    FrameNumber frame, const std::vector<Cluster> &clusters,
    const std::vector<std::shared_ptr<const ClusterDescriptor>> &descriptors) {
  for (size_t i = 0; i < descriptors.size(); i += 1) {
    int chainIndex = matches[i];
    if (chainIndex == -1) {
      ClusterChain chain;
      chainIndex = chains.size();
      chains.push_back(chain);
    }
    auto &chain = chains[chainIndex];
    chain.clusters()[frame] = &clusters[i];
    chain.descriptors()[frame] = descriptors[i];
  }
}

} // MouseTrack
