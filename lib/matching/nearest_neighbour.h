/* \file
 * nearest_neighbour.h
 *
 *  Created on: Mar 25, 2018
 *      Author: itto
 */

#pragma once

#include "matching.h"

namespace MouseTrack {

class NearestNeighbour: public Matching {
	public:
		/// Given a list of cluster chains, we find the best matching to append the given list of descriptors.
		/// descriptors: a list of descriptors from a new frame
		/// chains: a list of cluster chains to which we want to appeand one cluster from descriptors
		/// returns a vector of size descriptors.size(), which holds the index of the matching chain for each descriptor.
		/// If no match was found, the vector value is set to -1.
		virtual std::vector<long> operator()(const std::vector<std::shared_ptr<const ClusterDescriptor>>& descriptors, const std::vector<ClusterChain>& chains);
};

}
