/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include<vector>

namespace Mousetrack {


/// Interface for Clustering algorithms
class Clustering {
public:
	/// Takes a point cloud and splits it into clusters.
	virtual std::vector<Cluster> operator(const PointCloud& cloud) const = 0;
};


}
