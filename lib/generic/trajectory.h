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
	/// N x 3 matrix, each row holds the (x,y,z) coordinates of a 3D point.
	Eigen::MatrixXd& controlPoints();
	const Eigen::MatrixXd& controlPoints() const;
};


}
