/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include<vector>

namespace Mousetrack {

struct TrajectoryFrame {
	/// Think of it as the time `t` taking integer values
	size_t frameNumber;
	std::vector<std::shared_ptr<const ClusterDescriptor> clusters;
}

class TrajectoryBuilder {
public:
	/// Aggregates descriptors from different frames into trajectories.
	/// descriptors: a list of frames, holding the frame number and a list of descriptors
	/// trajectories: Descriptors are added to existing trajectories, if similar enough, new trajectories might be created
virtual void operator(const std::vector<TrajectoryFrame>& descriptors, std::vector<Trajectory>& trajectories) = 0;
};


}
