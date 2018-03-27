/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///


#include "mean_shift.h"
#include <stdlib.h>
#include <math.h>
#include <boost/log/trivial.hpp>
#include <Eigen/Dense>

namespace MouseTrack {

MeanShift::MeanShift(double window_size) : _window_size(window_size) {
    //empty
}

std::vector<Cluster> MeanShift::operator()(const PointCloud& cloud) const {

	std::vector<Cluster> clustering = std::vector<Cluster>();
	return clustering;
}

double MeanShift::apply_gaussian_kernel(const PointCloud& cloud, const PointIndex& idx, Eigen::MatrixXd mean) const {

	// Convert to Eigen type and check if dimensions match
	Eigen::MatrixXd point = cloud[idx].eigenVec();
	if (mean.size() != point.size()) BOOST_LOG_TRIVIAL(error) << "Vector dimensions don't match";

	// Euclidean distance squared btw. mean of kernel and point of interest
	const double d = (mean-point).squaredNorm();

	// Apply gaussian distribution
	return exp(-d/(2*_window_size));
}

void MeanShift::iterate() {
	//TODO
}
} //MouseTrack
