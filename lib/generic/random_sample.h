/// \file
/// Maintainer: Luzian Hug
///
///

#pragma once

#include "generic/point_cloud.h"

namespace MouseTrack {
/// Returns a random sample of size "size" of points from a point cloud
PointCloud random_sample(const PointCloud &cloud, const int size);

} // namespace MouseTrack
