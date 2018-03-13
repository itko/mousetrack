/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include<vector>

namespace Mousetrack {


class TrajectoryBuilder {
public:
	/// Aggregates descriptors from different frames into trajectories.
	virtual std::vector<Trajectory> operator(const std::vector<std::vector<std::unique_ptr<ClusterDescriptor>>>& descriptors) = 0;
};


}
