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
	// Covert point cloud to Eigen vectors
	std::vector<Eigen::VectorXd> points;
	for (int i=0; i<cloud.size(); i++) {
		points.push_back(cloud[i].eigenVec());
	}

	std::vector<Eigen::VectorXd> prev(points.size());
	std::vector<Eigen::VectorXd> curr = points;
	bool has_converged = false;

	while (!has_converged) {

		// perform 1 iteration on each mode
		for (int i=0; i<prev.size(); i++) {
			curr[i] = iterate_mode(prev[i], prev);
		}

		// Merge step
		// For every mode..
		for (int i=0; i<curr.size(); i++) {
			// Check modes we haven't already executed the (i)-loop for
			for (int j=i+1; i<curr.size(); j++) {
				Eigen::VectorXd diff = curr[i] - curr[j];
				if (diff.norm() < MERGE_THRESHOLD) {
					curr.erase(curr.begin() + j);
					break;
				}
			}
		}

		// Check convergence
		if (prev.size() == curr.size()) {
			double total_movement;
			for (int i=0; i < curr.size(); i++) {
				total_movement += (curr[i]-prev[i]).norm();
			}
			if (total_movement <= CONVERGENCE_THRESHOLD) has_converged = true;
		}
		prev = curr;
	}
	BOOST_LOG_TRIVIAL(trace) << "MeanShift converged! #Clusters: " << curr.size();

	// Build clusters
	std::vector<Cluster> clustering = std::vector<Cluster>();
	return clustering;
}

double MeanShift::apply_gaussian_kernel(const Eigen::VectorXd point, const Eigen::VectorXd mean) const {

	// check if dimensions match
	if (mean.size() != point.size()) BOOST_LOG_TRIVIAL(error) << "Vector dimensions don't match";

	// Euclidean distance squared btw. mean of kernel and point of interest
	const double d = (mean-point).squaredNorm();

	// Apply gaussian distribution
	return exp(-d/(2*_window_size));
}

Eigen::VectorXd MeanShift::iterate_mode(const Eigen::VectorXd mode, const std::vector<Eigen::VectorXd>& state) const {
	//COG Normalization Factor
	double normfact = 0;
	//Rest of COG
	Eigen::VectorXd cog = Eigen::VectorXd::Zero(mode.size());
	for(const Eigen::VectorXd x : state) {
		double temp = apply_gaussian_kernel(x,mode);
		normfact += temp;
		cog += temp * x;
	}
	return cog * (1.0/normfact);
}

} //MouseTrack
