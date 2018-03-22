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
    DisparityMap normalizedDepth;
    DisparityMap rawDepth;
    Picture leftPicture;
    Precision focallength;
    Precision baseline;
    Precision ccx;
    Precision ccy;
    Eigen::Matrix4d rotationCorrection;
    Eigen::Matrix4d camChainRotationDepth;
    Eigen::Matrix4d camChainRotationPicture;
};


} // MouseTrack
