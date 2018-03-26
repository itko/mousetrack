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

class RandomClustering: public Clustering {
public:

  /// k is the number of desired clusters
  RandomClustering(const int k);
  /// Splits a point cloud into k clusters randomly
  std::vector<Cluster> operator()(const PointCloud& cloud) const;

private:
    const int _k;
};

}
