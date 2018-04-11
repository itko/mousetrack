/** \file
 * nearest_neighbour.cpp
 *
 *  Created on: Mar 25, 2018
 *      Author: itto
 */

#include "nearest_neighbour.h"
#include "Eigen/Core"
#include <limits.h>
#include <boost/log/trivial.hpp>

namespace MouseTrack {

NearestNeighbour::~NearestNeighbour() {}

std::vector<long> NearestNeighbour::operator()(
    const std::vector<std::shared_ptr<const ClusterDescriptor>> &descriptors,
    const std::vector<ClusterChain> &chains) {
  // Prepare an empty vector that will store the index of each chain
  std::vector<long> matchIndex;
  for (size_t i = 0; i < descriptors.size(); i++) {
    BOOST_LOG_TRIVIAL(trace) << "Matching descriptor " << i;
    // Initialize a maximum distance and match index
    double nearest = std::numeric_limits<double>::max();
    long match = -1;
    for (size_t j = 0; j < chains.size(); j++) {
      // Check if we already assigned this chain
      if (std::find(matchIndex.begin(), matchIndex.end(), j) ==
          matchIndex.end()) {
        // We are only really concerned with the last cluster.
        // Get the frame number and the descriptor
        auto descriptor = chains[j].descriptors().rbegin();
        // Get the descriptor for that frame.
        auto lastDescriptor = descriptor->second;
        // Compare the descriptor to the cluster. If it's closer than the
        // previous chain then choose that one.
        double distance = descriptors[i]->compare(lastDescriptor.get());
        if (nearest > distance) {
          // Better match than previous chain. Pick that one.
          match = j;
          nearest = distance;
        }
      }
    }
    // Done comparing. Push.
    matchIndex.push_back(match);
  }
  return matchIndex;
};

} // namespace MouseTrack
