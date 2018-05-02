/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "types.h"

namespace MouseTrack {

// returns true if successful
bool write_png(const PictureI &img, const std::string &path);
bool write_png(const PictureD &img, const std::string &path);

} // namespace MouseTrack
