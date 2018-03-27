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
  // window size parameter for mean shift algorithm
  double _window_size;

  // contains the means of all the mean shift clusters
  PointCloud means;

  // Returns a weight in [0,1] for point by applying a gaussian kernel with variance window_size and mean mean
  double apply_gaussian_kernel(const Eigen::MatrixXd point, const Eigen::MatrixXd mean) const;

  // Performs one iteration of the mean shift algorithm
  void iterate();
};

}
