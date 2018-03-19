/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include <Eigen/Core>

namespace MouseTrack {


/// Stores the perspective depth for each pixel in a matrix.
class DisparityMap {
private:
    Eigen::MatrixXd _map;
public:
    /// Create empty map.
    DisparityMap();

    /// Copy input map.
    DisparityMap(const Eigen::MatrixXd& map);

    /// Consume input map
    DisparityMap(const Eigen::MatrixXd&& map);

    /// Access the depth map with the intention to modify.
	Eigen::MatrixXd& zMap();

    /// Access the depth map with the intention to only read.
	const Eigen::MatrixXd& zMap() const;
};


} // MouseTrack
