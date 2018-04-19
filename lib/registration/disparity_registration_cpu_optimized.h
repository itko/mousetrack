/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "disparity_registration.h"

namespace MouseTrack {

///
/// Tries to optimize the inner loops with eigen operations.
///
///
class DisparityRegistrationCpuOptimized : public DisparityRegistration {
public:
  virtual PointCloud operator()(const FrameWindow &window) const;
};

} // namespace MouseTrack
