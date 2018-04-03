/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "registration.h"

namespace MouseTrack {

class DisparityRegistration : public Registration {
public:
    PointCloud operator()(const FrameWindow& window) const;
private:
    /// constant from fpga set up
    double min_disparity = 1*2+32;
    /// constant from fpga set up
    int xshift = 22;
    /// constant from fpga set up
    int yshift = -8;
    /// constant from fpga set up
    int frame_border = 80;
};

} // MouseTrack
