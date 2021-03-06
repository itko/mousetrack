/// \file
/// Maintainer: Luzian Hug
/// Created: 23.04.2018
///
///

#pragma once

#include "clustering.h"
#include "generic/cluster.h"
#include <vector>


namespace MouseTrack {

/// Throws all the points in a single cluster.
/// This exists for debug purposes or special use cases.
class SingleCluster : public Clustering {
public:

  SingleCluster();

  std::vector<Cluster> operator()(const PointCloud &cloud) const;

};

} // namespace MouseTrack
