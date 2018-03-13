/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include<vector>
#include<Eigen/Core>

namespace Mousetrack {


// stores the perspective depth for each pixel in a matrix
class DisparityMap {
	Eigen::MatrixXd& zMap();
	const Eigen::MatrixXd& zMap() const;
};


}
