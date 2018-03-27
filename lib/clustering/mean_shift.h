/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///

#pragma once

#include "clustering.h"
#include <vector>
#include "generic/cluster.h"
#include <Eigen/Core>

namespace MouseTrack {

class MeanShift: public Clustering {
public:

  /// k is the number of desired clusters
  MeanShift(double window_size);
  /// Splits a point cloud into k clusters randomly
  std::vector<Cluster> operator()(const PointCloud& cloud) const;

private:
  // when two peaks are closer than this, they are merged.
  const double MERGE_THRESHOLD = 0.0001;

  // Convergence is decided if modes haven't moved more than this in total
  const double CONVERGENCE_THRESHOLD = 0.1;

  // If algorithm hasn't converged after this amount of iterations, we abort.
  const int MAX_ITERATIONS = 100;

  // window size parameter for mean shift algorithm
  double _window_size;

  // contains the means of all the mean shift clusters
  PointCloud means;

  // Returns a weight in [0,1] for point by applying a gaussian kernel with variance window_size and mean mean
  double apply_gaussian_kernel(const Eigen::VectorXd point, const Eigen::VectorXd mean) const;

  // Performs one iteration of the mean shift algorithm for a single mode
  Eigen::VectorXd iterate_mode(const Eigen::VectorXd mode, const std::vector<Eigen::VectorXd>& state) const;
};

}
