/* \file
 * cog.cpp
 *
 *  Created on: Mar 25, 2018
 *      Author: itto
 */

#include "cog.h"
#include "generic/center.h"

namespace MouseTrack {

/// Takes a cluster and its referencing PointCloud and creates a cluster descriptor
std::unique_ptr<ClusterDescriptor> CenterOfGravity::operator()(const Cluster& cluster, const PointCloud& cloud) const {
	std::unique_ptr<ClusterDescriptor> descriptor(Center());
	return descriptor;
};

} // MouseTrack


