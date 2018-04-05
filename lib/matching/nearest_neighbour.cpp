/** \file
 * nearest_neighbour.cpp
 *
 *  Created on: Mar 25, 2018
 *      Author: itto
 */

#include "nearest_neighbour.h"
#include "Eigen/Core"
#include <limits.h>

namespace MouseTrack {

std::vector<long> NearestNeighbour::operator()(
    const std::vector<std::shared_ptr<const ClusterDescriptor>> &descriptors,
    const std::vector<ClusterChain> &chains) {
  // Prepare an empty vector that will store the index of each chain
  std::vector<long> matchIndex;
  for (size_t j = 0; j < descriptors.size(); j++) {
    // Initialize a maximum distance
    double nearest = std::numeric_limits<double>::max();
    for (size_t i = 0; i < chains.size(); i++) {
      // In this case we are only really concerned with the last cluster. Get
      // the frame.
      FrameNumber lastFrameNumber = chains[i].descriptors().rbegin()->first;
      // Get the cluster for that frame.
      std::shared_ptr<const ClusterDescriptor> lastDescriptor =
          chains[i].descriptors().find(lastFrameNumber)->second;
      // Compare the descriptor to the cluster. If it's closer than the previous
      // chain then choose that one.
      // TODO This means that there may be repeated descriptors pointing to the
      // same cluster chain.
      long match = 0;
      if (nearest > descriptors[j]->compare(lastDescriptor.get())) {
        // Better match than previous chain. Pick that one.
        match = i;
      }
      // Done comparing. Push.
      matchIndex.push_back(match);
    }
  }
  return matchIndex;
};

} // namespace MouseTrack
