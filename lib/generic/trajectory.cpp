/// \file
/// Maintainer: Felice Serena
///
///

#include "trajectory.h"


namespace MouseTrack {

std::map<size_t, std::shared_ptr<Eigen::Vector3d>> Trajectory::controlPoints() {
    return _controlPoints;
}

const std::map<size_t, std::shared_ptr<Eigen::Vector3d>> Trajectory::controlPoints() const {
    return _controlPoints;
}


} // MouseTrack
