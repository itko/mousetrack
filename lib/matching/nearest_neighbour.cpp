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
  for (size_t i = 0; i < chains.size(); i++) {
    // Initialize a maximum distance
    double nearest = std::numeric_limits<double>::max();
    // In this case we are only really concerned with the last cluster.
    // Get the frame number and the descriptor
    auto descriptor = chains[i].descriptors().rbegin();
    // Get the descriptor for that frame.
    auto lastDescriptor = descriptor->second;
    for (size_t j = 0; j < descriptors.size(); j++) {
      // Compare the descriptor to the cluster. If it's closer than the previous
      // chain then choose that one.
      // TODO This means that there may be multipel chains pointing to the
      // same descriptor.
      long match = 0;
      double distance = descriptors[j]->compare(lastDescriptor.get());
      if (nearest > distance) {
        // Better match than previous chain. Pick that one.
        match = i;
        nearest = distance;
      }
      // Done comparing. Push.
      matchIndex.push_back(match);
    }
  }
  return matchIndex;
};

} // namespace MouseTrack
