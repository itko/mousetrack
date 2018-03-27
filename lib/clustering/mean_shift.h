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


  // computes squared euclidean distance ||x-y||. x and y are 4D (x,y,z,intensity)
  double euclidean_distance_squared(const Point& x, const Point& y);

  /// Returns a weight in [0,1] for point "point" in "cloud" by applying gaussian kernel "kernel"
  double apply_gaussian_kernel(const PointCloud& cloud, const PointIndex& point, const GaussianKernel& kernel);

};

/// Defines a 4D gaussian kernel with mean and variance (variance is equal in each direction)
struct GaussianKernel {
  Point mean;
  double variance;
};

}
