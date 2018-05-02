/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "types.h"

namespace MouseTrack {

std::string resolve_symlink(const std::string &p, int maxSymlinksFollowed);

} // namespace MouseTrack
