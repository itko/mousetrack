/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "generic/point_cloud.h"

namespace MouseTrack {

/// Writes a point cloud as PLY file to `path`
void write_point_cloud(const std::string &path, const PointCloud &cloud);


/// Writes some point cloud metrics as csv file to `path`
void write_point_cloud_metrics(const std::string &path,
                               const PointCloud &cloud);
} // namespace MouseTrack
