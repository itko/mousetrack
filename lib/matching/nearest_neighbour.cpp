/** \file
 * nearest_neighbour.cpp
 *
 *  Created on: Mar 25, 2018
 *      Author: itto
 */

#include "nearest_neighbour.h"
#include "Eigen/Core"

namespace MouseTrack {

std::vector<long> NearestNeighbour::operator()(const std::vector<std::shared_ptr<const ClusterDescriptor>>& descriptors, const std::vector<ClusterChain>& chains) {
	for (size_t i = 0; i<chains.size(); i++) {
		Eigen::VectorXd compare(descriptors.size());
		// In this case we are only really concerned with the last cluster. Get the frame.
		FrameNumber lastFrameNumber = chains[i].descriptors().rbegin()->first;
		// Get the cluster for that frame.
		std::shared_ptr<const ClusterDescriptor> lastDescriptor = chains[i].descriptors().find(lastFrameNumber)->second;
		// Go through each descriptor
		for (size_t j = 0; j<descriptors.size(); j++) {
			// Compare the descriptor to the cluster
			compare(j) = descriptors[j]->compare(lastDescriptor.get());
		}
		// Assign the descriptor that is closest to our current cluster
		// TODO This means that there may be repeated descriptors for clusters.
	}
	return std::vector<long>{};
};

} // MouseTrack


