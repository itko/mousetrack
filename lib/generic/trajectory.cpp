/// \file
/// Maintainer: Felice Serena
///
///

#include "trajectory.h"

namespace MouseTrack {

std::map<FrameNumber, Eigen::Vector3d> Trajectory::controlPoints() {
  return _controlPoints;
}

const std::map<FrameNumber, Eigen::Vector3d> Trajectory::controlPoints() const {
  return _controlPoints;
}

} // namespace MouseTrack
