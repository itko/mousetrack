/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "generic/point_cloud.h"

namespace MouseTrack {

class PointCloudFiltering {
public:
    virtual ~PointCloudFiltering() = default;
    virtual PointCloud operator()(const PointCloud& inCloud) const = 0;
};

} // MouseTrack
