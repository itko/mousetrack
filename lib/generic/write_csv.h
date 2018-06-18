/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include <fstream>
#include <vector>

namespace MouseTrack {


/// Writes a csv file to `path` where `payload[row][col]` is assumed
template <typename T>
void write_csv(const std::string &path,
               const std::vector<std::vector<T>> &payload) {
  std::ofstream out;
  out.open(path.c_str());
  for (const auto &p : payload) {
    if (!p.empty()) {
      out << p[0];
    }
    for (size_t j = 1; j < p.size(); j += 1) {
      out << ",";
      out << p[j];
    }
    out << "\n";
  }
  out.close();
}

} // namespace MouseTrack
