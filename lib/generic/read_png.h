/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "types.h"

namespace MouseTrack {

Eigen::MatrixXi read_png(const std::string &path);
Picture read_png_normalized(const std::string &path);

} // namespace MouseTrack
