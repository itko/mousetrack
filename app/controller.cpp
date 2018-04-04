/// \file
/// Maintainer: Felice Serena
///



#include "controller.h"

namespace MouseTrack {

Pipeline& Controller::pipeline() {
    return _pipeline;
}

const Pipeline& Controller::pipeline() const {
    return _pipeline;
}

} // MouseTrack
