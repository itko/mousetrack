/// \file
/// Maintainer: Luzian Hug
/// Created: 25.03.2018
///
///

#pragma once

#include "clustering.h"
#include <vector>

namespace MouseTrack {

class RandomClustering: public Clustering {
public:
    RandomClustering(const int k);

    ///Splits a point cloud into k clusters randomly
    std::vector<Cluster> operator()(const PointCloud& cloud);

private:
    const int _k;
};

}
