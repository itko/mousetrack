/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include<vector>
#include<Eigen/Core>

namespace Mousetrack {


/// Describes a path in 3D space
class Trajectory {
public:
	std::map<size_t, std::shared_ptr<const ClusterDescriptor>> controlPoints;
};


}
