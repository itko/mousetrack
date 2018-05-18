/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "types.h"

namespace MouseTrack {

PictureI read_png(const std::string &path);
PictureD read_png_normalized(const std::string &path);

} // namespace MouseTrack
