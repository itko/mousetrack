/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include <string>
#include <vector>

namespace MouseTrack {

/// Takes a string and a delemiter character and splits the string at each delemiter character
/// Returns the strings between delemiters as a vector
std::vector<std::string> explode(const std::string& str, char delemiter);

} // MouseTrack
