/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "disparity_registration.h"

namespace MouseTrack {

///
/// Simple, sequential, straight forward solution.
/// Think of it as the most basic implementation suitable as reference.
///
///
class DisparityRegistrationSimple : public DisparityRegistration {
public:
  virtual PointCloud operator()(const FrameWindow &window) const;
};

} // namespace MouseTrack
