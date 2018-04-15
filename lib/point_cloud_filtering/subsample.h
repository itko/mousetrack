/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "point_cloud_filtering.h"

namespace MouseTrack {

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
