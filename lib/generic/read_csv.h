/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include <string>
#include <vector>

namespace MouseTrack {

std::vector<std::vector<std::string>> read_csv(const std::string& file, char col_delemiter = ',', char row_delemiter = '\n');

} // MouseTrack
