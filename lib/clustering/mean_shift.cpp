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
#include <iostream>

namespace MouseTrack {

MeanShift::MeanShift(double window_size) : _window_size(window_size) {
	//empty
}

std::vector<Cluster> MeanShift::operator()(const PointCloud& cloud) const {
	std::vector<Cluster> clustering = std::vector<Cluster>();
	return clustering;
}

double MeanShift::apply_gaussian_kernel(const Eigen::MatrixXd point, const Eigen::MatrixXd mean) const {

	// check if dimensions match
	if (mean.size() != point.size()) BOOST_LOG_TRIVIAL(error) << "Vector dimensions don't match";

	// Euclidean distance squared btw. mean of kernel and point of interest
	const double d = (mean-point).squaredNorm();

	// Apply gaussian distribution
	return exp(-d/(2*_window_size));
}

} //MouseTrack
