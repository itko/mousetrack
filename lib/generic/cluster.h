/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include<vector>

namespace Mousetrack {

/// Describes a cluster within a PointCloud. Nothing more than a list of indexes to 3D points in the PointCloud.
class Cluster {
public:
	std::vector<size_t>& points();
	const std::vector<size_t>& points() const;
};

}
