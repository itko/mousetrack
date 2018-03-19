/// \file
/// Maintainer: Felice Serena
///
///

#include "cluster.h"


namespace MouseTrack {

Cluster::Cluster(const std::vector<size_t>& points) : _points(points) {
    // empty
}

Cluster::Cluster(std::vector<size_t>&& points) : _points(std::move(points)) {
    // empty
}

std::vector<size_t>& Cluster::points() {
    return _points;
}
const std::vector<size_t>& Cluster::points() const {
    return _points;
}

} // MouseTrack

