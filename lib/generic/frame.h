/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "generic/types.h"
#include "generic/disparity_map.h"
#include <Eigen/Core>

namespace MouseTrack {


/// Holds all information related to a single frame of a stream
struct Frame {
    DisparityMap normalizedDisparityMap;
    DisparityMap rawDisparityMap;
    Picture referencePicture;
    Precision focallength;
    Precision baseline;
    Precision ccx;
    Precision ccy;
    Eigen::Matrix4d rotationCorrection;
    Eigen::Matrix4d camChainPicture;
    Eigen::Matrix4d camChainDisparity;
};


} // MouseTrack
