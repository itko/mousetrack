/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "types.h"

namespace MouseTrack {

/// Read a png from `path` and convert it to int representation (intensities in [0,255])
PictureI read_png(const std::string &path);

/// Read a pong form `path` and convert it to double representation (intensities in [0,1])
PictureD read_png_normalized(const std::string &path);

} // namespace MouseTrack
