/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "types.h"

namespace MouseTrack {

/// follows symlinks `maxSymlinksFollowed` times and returns the found path
std::string resolve_symlink(const std::string &p, int maxSymlinksFollowed);

} // namespace MouseTrack
