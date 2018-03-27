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
  /// Defines a 4D gaussian kernel with mean and variance (variance is equal in each direction)
  struct GaussianKernel {
    Point mean;
    double variance;
  };
  
  double apply_gaussian_kernel(const PointCloud& cloud, const PointIndex& point, const GaussianKernel& kernel);

};


}
