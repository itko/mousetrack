/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///

#pragma once

#include "clustering.h"
#include <vector>
#include "generic/cluster.h"

namespace MouseTrack {

class MeanShift: public Clustering {
public:

  /// k is the number of desired clusters
  MeanShift(const int k);
  /// Splits a point cloud into k clusters randomly
  std::vector<Cluster> operator()(const PointCloud& cloud) const;

private:
  const int _k;

  // window size parameter for mean shift algorithm
  const double window_size;

  // contains the means of all the mean shift clusters
  PointCloud means;

  // Returns a weight in [0,1] for point w/ index "point" in "cloud" by applying a gaussian kernel with variance window_size and mean mean
  double apply_gaussian_kernel(const PointCloud& cloud, const PointIndex& point, const Eigen::Vector4d mean);

  // Performs one iteration of the mean shift algorithm
  void iterate();
};

}
