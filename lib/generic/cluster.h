/// \file
/// Maintainer: Felice Serena
///
///

#pragma once


#include "types.h"
#include <vector>
#include "Eigen/Core"
#include "point_cloud.h"

namespace MouseTrack {

/// Describes a cluster within a PointCloud. Nothing more than a list of indexes to 3D points in the PointCloud.
/// The corresponding point cloud is not linked in this data structure.
class Cluster {
public:
    /// Construct empty cluster object
    Cluster();

    /// Construct a new cluster with a list of indices.
    Cluster(const std::vector<PointIndex>& points);

    /// Construct a new cluster with a list of indices.
    Cluster(std::vector<PointIndex>&& points);

    /// Read-Write access to indices
    std::vector<PointIndex>& points();

    /// Read access to indices.
    const std::vector<PointIndex>& points() const;

    /// Get center of gravity
    Eigen::VectorXd center_of_gravity(PointCloud& cloud);

private:
    std::vector<PointIndex> _points;
};

} // MouseTrack
