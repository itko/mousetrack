/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///

#pragma once

#include "clustering.h"
#include "generic/cluster.h"
#include <Eigen/Core>
#include <vector>


namespace MouseTrack {
class KMeans : public Clustering {
public:
  /// k is the number of desired clusters
  KMeans(int k);
  /// Splits a point cloud into k clusters according to the K-Means algorithm
  std::vector<Cluster> operator()(const PointCloud &cloud) const;

  void max_iterations(int max_iterations);
  const int max_iterations() const;

  void epsilon(double epsilon);
  const double epsilon() const;

  void attempts(int attempts);
  const int attempts() const;
private:
    int _k;

    // Algorithm terminates if max_iterations is reached OR the cluster centers move less than epsilon in an iteration.
    int _max_iterations = 100;
    double _epsilon = 0.01;

    // The algorithm will execute this number of times and return the most compact result
    int _attempts = 3;
};

} // namespace MouseTrack
