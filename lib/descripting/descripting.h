/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "generic/cluster.h"
#include "generic/cluster_descriptor.h"
#include "generic/point_cloud.h"
#include <memory>
#include <vector>

namespace MouseTrack {

class Descripting {
	public:
		/// Takes a cluster and its referencing PointCloud and creates a cluster descriptor
        virtual std::unique_ptr<ClusterDescriptor> operator()(const Cluster& cluster, const PointCloud& cloud) const = 0;
};


} // MouseTrack
