/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "types.h"

namespace MouseTrack {

/// Writes an image to `path`
///
/// returns true if successful
bool write_png(const PictureI &img, const std::string &path);

/// Writes an image to `path`
bool write_png(const PictureD &img, const std::string &path);

} // namespace MouseTrack
