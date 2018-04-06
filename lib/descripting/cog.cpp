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
std::unique_ptr<ClusterDescriptor> CenterOfGravity::operator()(
    const Cluster &cluster, const PointCloud &cloud) const {
  Coordinate sumX = 0, sumY = 0, sumZ = 0;
  size_t numPoints = cluster.points().size();
  // Go through each point in the cluster
  for (PointIndex i = 0; i < cluster.points().size(); i++) {
    // Get the sum of all coordinates along x, y ,z
    sumX += cloud[i].x();
    sumY += cloud[i].y();
    sumZ += cloud[i].z();
  }
  // Divide to get average x, y, z. This is the center of gravity.
  std::unique_ptr<Center> descriptor(
      new Center(sumX / numPoints, sumY / numPoints, sumZ / numPoints));
  return descriptor;
};

}  // namespace MouseTrack
