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
	// Convert point cloud to Eigen vectors
	std::vector<Eigen::VectorXd> points(cloud.size());
	for (PointIndex i=0; i<cloud.size(); i++) {
		points[i] = cloud[i].eigenVec();
	}


	// Initialize Clusters. Initially, every point has its own cluster.
	std::vector<Cluster> clusters(cloud.size());
	for (PointIndex i=0; i<cloud.size(); i++) {
		 clusters[i].points().push_back(i);

	}

	// Initialize some stuff used in the MeanShift loop
	Eigen::VectorXd prevCenter;
	std::vector<Eigen::VectorXd> currCenters = points;

	// For each point...
	for(PointIndex i = 0; i < clusters.size(); i++) {
		prevCenter = points[i];
		int iterations = 0; //for logging and abort condition

		// ... iterate until convergence
		do  {
			iterations++;
			// perform one iteration of mean shift
			prevCenter = currCenters[i];
			currCenters[i] = iterate_mode(currCenters[i], points);

			if (iterations > _max_iterations) {
				BOOST_LOG_TRIVIAL(warning) << "Max number of iterations for point " << i << " reached - continuing without convergence for this point";
			}
		} while ((prevCenter - currCenters[i]).norm() > _convergence_threshold);

	}

	// Merge step
	// For every mode..
	for (int i=0; i<currCenters.size(); i++) {
		// Check modes we haven't already executed the (i)-loop for
		for (int j=i+1; j < currCenters.size(); j++) {
			Eigen::VectorXd diff = currCenters[i] - currCenters[j];
			if (diff.norm() < _merge_threshold) {
				// Merge clusters. Erase one of the modes corresponding to the clusters and append points belonging to j to cluser of i
				clusters[i].points().insert(clusters[i].points().begin(), clusters[j].points().begin(), clusters[j].points().end());
				clusters.erase(clusters.begin() + j);
				currCenters.erase(currCenters.begin() + j);
				// We need to decrement i here because there might be another cluster that wants to merge with i.
				i--;
				break;
			}
		}
	}

	BOOST_LOG_TRIVIAL(trace) << "MeanShift converged! #Clusters: " << clusters.size();
	return clusters;
}

double MeanShift::gaussian_weight(const Eigen::VectorXd point, const Eigen::VectorXd mean) const {

	// assert if dimensions match
	assert(mean.size() == point.size());

	// Euclidean distance squared btw. mean of kernel and point of interest
	const double d = (mean-point).squaredNorm();

	// Apply gaussian distribution
	return exp(-d/(2*_window_size));
}

Eigen::VectorXd MeanShift::iterate_mode(const Eigen::VectorXd mode, const std::vector<Eigen::VectorXd>& fixedPoints) const {
	//COG Normalization Factor
	double normfact = 0;
	//Rest of COG
	Eigen::VectorXd cog = Eigen::VectorXd::Zero(mode.size());
	for(int i; i<fixedPoints.size(); i++) {
		double temp = gaussian_weight(fixedPoints[i],mode);
		normfact += temp;
		cog += temp * fixedPoints[i];
	}
	cog = cog * (1.0/normfact);
	return cog;
}


void MeanShift::setMaxIterations(int max_iterations) {
	_max_iterations = max_iterations;
}

int MeanShift::getMaxIterations() const {
	return _max_iterations;
}

void MeanShift::setMergeThreshold(double merge_threshold){
	_merge_threshold = merge_threshold;
}
double MeanShift::getMergeThreshold() const {
	return _merge_threshold;
}

void MeanShift::setConvergenceThreshold(double convergence_threshold) {
	_convergence_threshold = convergence_threshold;
}
double MeanShift::getConvergenceThreshold() const {
	return _convergence_threshold;
}

} //MouseTrack
