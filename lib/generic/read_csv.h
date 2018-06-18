/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include <string>
#include <vector>

namespace MouseTrack {


/// reads a csv file from `file` assuming `col_delemiter` and `row_delemiter` as seprations symbols
std::vector<std::vector<std::string>> read_csv(const std::string &file,
                                               char col_delemiter = ',',
                                               char row_delemiter = '\n');

} // namespace MouseTrack
