/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///


#include "mean_shift.h"
#include <boost/log/trivial.hpp>
#include <Eigen/Dense>

namespace MouseTrack {

MeanShift::MeanShift(double window_size) : _window_size(window_size) {
	//empty
}

std::vector<Cluster> MeanShift::operator()(const PointCloud& cloud) const {
	BOOST_LOG_TRIVIAL(trace) << "MeanShift algorithm started";
	// Covert point cloud to Eigen vectors
	std::vector<Eigen::VectorXd> points(cloud.size());
	for (int i=0; i<cloud.size(); i++) {
		points[i] = cloud[i].eigenVec();
	}


	// Initialize Clusters. Initially, every point has its own cluster.
	std::vector<Cluster> clusters(cloud.size());
	for (PointIndex i=0; i<cloud.size(); i++) {
		 clusters[i].points().push_back(i);

	}

	// Initialize some stuff used in the MeanShift loop
	std::vector<Eigen::VectorXd> prev = points;
	std::vector<Eigen::VectorXd> curr(points.size());
	bool has_converged = false;
  int iterations = 0; //for logging

	// The MeanShift loop
	while (!has_converged) {
		iterations++;
		if (iterations > MAX_ITERATIONS) {
			BOOST_LOG_TRIVIAL(error) << "MeanShift: max number of iterations exceeded without convergence";
			break;
		}
		// perform 1 iteration on each mode
		for (int i=0; i<prev.size(); i++) {
			curr[i] = iterate_mode(prev[i], prev);
		}

		// Merge step
		// For every mode..
		for (int i=0; i<curr.size(); i++) {
			// Check modes we haven't already executed the (i)-loop for
			for (int j=i+1; j<curr.size(); j++) {
				Eigen::VectorXd diff = curr[i] - curr[j];

				if (diff.norm() < MERGE_THRESHOLD) {
					// Merge clusters. Erase one of the modes corresponding to the clusters and append points belonging to j to cluser of i
					for (int k=0; k < clusters[j].points().size(); k++) {
						clusters[i].points().push_back(clusters[j].points()[k]);
					}
					//BOOST_LOG_TRIVIAL(debug) << "Merging cluster " << j << " into cluster " << i << ", distance " << diff.norm();
					clusters.erase(clusters.begin() + j);
					curr.erase(curr.begin() + j);

					break;
				}
			}
		}

		if (clusters.size() != curr.size())
			BOOST_LOG_TRIVIAL(warning) << "Cluster count does not equal mode count. (" << clusters.size() << " != " << curr.size() << ")";

		// Check convergence
		if (prev.size() == curr.size()) {
			double total_movement = 0;
			for (int i=0; i < curr.size(); i++) {
				total_movement += (curr[i]-prev[i]).norm();
			}
			if (total_movement <= CONVERGENCE_THRESHOLD) has_converged = true;
		}
		prev = curr;
	}
	BOOST_LOG_TRIVIAL(trace) << "MeanShift converged! #Clusters: " << curr.size() << " #Iterations: " << iterations;


	return clusters;
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
	for(int i; i<state.size(); i++) {
		double temp = apply_gaussian_kernel(state[i],mode);
		normfact += temp;
		cog += temp * state[i];
	}
	cog = cog * (1.0/normfact);
	return cog;
}

} //MouseTrack
