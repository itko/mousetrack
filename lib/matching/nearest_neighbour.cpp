/* \file
 * nearest_neighbour.cpp
 *
 *  Created on: Mar 25, 2018
 *      Author: itto
 */

#include "nearest_neighbour.h"

namespace MouseTrack {

std::vector<long> NearestNeighbour::operator()(const std::vector<std::shared_ptr<const ClusterDescriptor>>& descriptors, const std::vector<ClusterChain>& chains) {
	return std::vector<long>{};
};

} // MouseTrack


