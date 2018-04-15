/// \file
/// Maintainer: Felice Serena
///
///

#include "disparity_map.h"

namespace MouseTrack {

DisparityMap::DisparityMap() {
  // empty
}

DisparityMap::DisparityMap(const Eigen::MatrixXd &map) : _map(map) {
  // empty
}

DisparityMap::DisparityMap(const Eigen::MatrixXd &&map) : _map(std::move(map)) {
  // empty
}

Eigen::MatrixXd &DisparityMap::zMap() { return _map; }

const Eigen::MatrixXd &DisparityMap::zMap() const { return _map; }

} // namespace MouseTrack
