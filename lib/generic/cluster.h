/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include <cstddef>
#include <vector>

namespace MouseTrack {

/// Describes a cluster within a PointCloud. Nothing more than a list of indexes to 3D points in the PointCloud.
/// The corresponding point cloud is not linked in this data structure.
class Cluster {
public:
    /// Construct a new cluster with a list of indices.
    Cluster(const std::vector<size_t>& points);
    /// Construct a new cluster with a list of indices.
    Cluster(std::vector<size_t>&& points);
    /// Read-Write access to indices
	std::vector<size_t>& points();
    /// Read access to indices.
	const std::vector<size_t>& points() const;
private:
    std::vector<size_t> _points;
};

} // MouseTrack
