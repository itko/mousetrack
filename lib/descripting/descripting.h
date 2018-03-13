/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include<vector>

namespace Mousetrack {


class Descripting {
	public:
		/// Takes a cluster and its referencing PointCloud and creates a cluster descriptor
		virtual std::unique_ptr<ClusterDescriptor> operator(const Cluster& clusters, const PointCloud& cloud) const = 0;
};


}
