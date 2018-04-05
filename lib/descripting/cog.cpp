/** \file
 * cog.cpp
 *
 *  Created on: Mar 25, 2018
 *      Author: itto
 */

#include "cog.h"
#include "generic/center.h"

namespace MouseTrack {

/// Takes a cluster and its referencing PointCloud and creates a cluster
/// descriptor
std::unique_ptr<ClusterDescriptor> CenterOfGravity::
operator()(const Cluster &cluster, const PointCloud &cloud) const {
  Coordinate sumX = 0, sumY = 0, sumZ = 0;
  size_t numPoints = cluster.points().size();
  for (PointIndex i = 0; i < cluster.points().size(); i++) {
    sumX += cloud[i].x();
    sumY += cloud[i].y();
    sumZ += cloud[i].z();
  }
  std::unique_ptr<ClusterDescriptor> descriptor(
      new Center(sumX / numPoints, sumY / numPoints, sumZ / numPoints));
  return descriptor;
};

} // namespace MouseTrack
