/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "point_cloud_filtering.h"

namespace MouseTrack {

/// Samples points randomly (without repetition) from a point cloud.
///
/// This creates a new point cloud that holds less data, but has the same statistical properties as the old point cloud.
class SubSample : public PointCloudFiltering {
public:
  SubSample() = default;
  SubSample(int desiredMaxPoints);
  virtual ~SubSample() = default;
  virtual PointCloud operator()(const PointCloud &inCloud) const;
  void desiredMaxPoints(int _new);
  int desiredMaxPoints() const;

private:
  int _desiredMaxPoints = 10000;
};

} // namespace MouseTrack
