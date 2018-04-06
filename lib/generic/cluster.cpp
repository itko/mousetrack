/// \file
/// Maintainer: Felice Serena
///
///

#include "cluster.h"


namespace MouseTrack {

Cluster::Cluster() : _points(std::vector<PointIndex>()){
    //empty
}

Cluster::Cluster(const std::vector<PointIndex>& points) : _points(points) {
    // empty
}

Cluster::Cluster(std::vector<PointIndex>&& points) : _points(std::move(points)) {
    // empty
}

std::vector<PointIndex>& Cluster::points() {
    return _points;
}
const std::vector<PointIndex>& Cluster::points() const {
    return _points;
}

Eigen::VectorXd Cluster::center_of_gravity(PointCloud& cloud) {
  Eigen::Vector4d cog (0,0,0,0);
  for(size_t i = 0; i < points().size(); i++) {
    cog += cloud[points()[i]].eigenVec();
  }
  cog /= points().size();
  return cog;
}

} // MouseTrack
